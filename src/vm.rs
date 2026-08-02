//!
//! Virtual Machine management module
//!

use crate::asm;
use crate::drivers::block_device::BlockDevice;
use crate::drivers::{
    generic_timer,
    gicv2::{self as physical_gicv2, GicDistributor, GicHypervisorInterface},
};
use crate::exception::Registers;
use crate::fat32::Fat32;
use crate::lock::Mutex;
use crate::mmio::{
    gicv2::GicDistributorMmio, pl011::Pl011Mmio, virtio_blk::VirtioBlkMmio,
    virtio_net::VirtioNetMmio,
};
use crate::paging::*;
use crate::registers::*;
use crate::vgic;

use core::marker::Send;
use core::sync::atomic::{AtomicUsize, Ordering};

use alloc::collections::linked_list::LinkedList;
use alloc::sync::Arc;
use alloc::vec::Vec;

pub trait MmioHandler {
    fn read(&mut self, offset: usize, access_width: u64) -> Result<u64, ()>;
    fn write(&mut self, offset: usize, access_width: u64, value: u64) -> Result<(), ()>;
}

pub struct MmioEntry {
    base_address: usize,
    length: usize,
    handler: Arc<Mutex<dyn MmioHandler + Send>>,
}

/// Snapshot of every guest-visible EL0/EL1 system register that is banked
/// per Exception Level by the hardware rather than per-VM (see
/// `asm::el1_register_accessors`): MMU/cache configuration, the exception
/// vector base, banked stack pointers, and the virtual timer comparator.
/// Part of `VcpuContext` because a cooperative VCPU switch (see
/// `try_yield_to_next_vcpu`) must fully replace this state, or the
/// incoming VCPU would keep running with the outgoing VCPU's MMU
/// translation tables and immediately fetch garbage.
#[derive(Clone, Copy)]
struct El1Context {
    sctlr_el1: u64,
    ttbr0_el1: u64,
    ttbr1_el1: u64,
    tcr_el1: u64,
    mair_el1: u64,
    amair_el1: u64,
    vbar_el1: u64,
    cpacr_el1: u64,
    contextidr_el1: u64,
    esr_el1: u64,
    far_el1: u64,
    par_el1: u64,
    afsr0_el1: u64,
    afsr1_el1: u64,
    tpidr_el0: u64,
    tpidr_el1: u64,
    sp_el0: u64,
    sp_el1: u64,
    elr_el1: u64,
    spsr_el1: u64,
    cntkctl_el1: u64,
    cntv_ctl_el0: u64,
    cntv_cval_el0: u64,
}

impl El1Context {
    /// Reads every register covered by `El1Context` from hardware, as it
    /// currently stands for whichever VCPU is physically running.
    fn capture_current() -> Self {
        Self {
            sctlr_el1: asm::get_sctlr_el1(),
            ttbr0_el1: asm::get_ttbr0_el1(),
            ttbr1_el1: asm::get_ttbr1_el1(),
            tcr_el1: asm::get_tcr_el1(),
            mair_el1: asm::get_mair_el1(),
            amair_el1: asm::get_amair_el1(),
            vbar_el1: asm::get_vbar_el1(),
            cpacr_el1: asm::get_cpacr_el1(),
            contextidr_el1: asm::get_contextidr_el1(),
            esr_el1: asm::get_esr_el1(),
            far_el1: asm::get_far_el1(),
            par_el1: asm::get_par_el1(),
            afsr0_el1: asm::get_afsr0_el1(),
            afsr1_el1: asm::get_afsr1_el1(),
            tpidr_el0: asm::get_tpidr_el0(),
            tpidr_el1: asm::get_tpidr_el1(),
            sp_el0: asm::get_sp_el0(),
            sp_el1: asm::get_sp_el1(),
            elr_el1: asm::get_elr_el1(),
            spsr_el1: asm::get_spsr_el1(),
            cntkctl_el1: asm::get_cntkctl_el1(),
            cntv_ctl_el0: asm::get_cntv_ctl_el0(),
            cntv_cval_el0: asm::get_cntv_cval_el0(),
        }
    }

    /// Programs hardware with this snapshot. Must be called before
    /// resuming (`eret`ing into) the VCPU this snapshot belongs to.
    fn activate(&self) {
        unsafe {
            asm::set_sctlr_el1(self.sctlr_el1);
            asm::set_ttbr0_el1(self.ttbr0_el1);
            asm::set_ttbr1_el1(self.ttbr1_el1);
            asm::set_tcr_el1(self.tcr_el1);
            asm::set_mair_el1(self.mair_el1);
            asm::set_amair_el1(self.amair_el1);
            asm::set_vbar_el1(self.vbar_el1);
            asm::set_cpacr_el1(self.cpacr_el1);
            asm::set_contextidr_el1(self.contextidr_el1);
            asm::set_esr_el1(self.esr_el1);
            asm::set_far_el1(self.far_el1);
            asm::set_par_el1(self.par_el1);
            asm::set_afsr0_el1(self.afsr0_el1);
            asm::set_afsr1_el1(self.afsr1_el1);
            asm::set_tpidr_el0(self.tpidr_el0);
            asm::set_tpidr_el1(self.tpidr_el1);
            asm::set_sp_el0(self.sp_el0);
            asm::set_sp_el1(self.sp_el1);
            asm::set_elr_el1(self.elr_el1);
            asm::set_spsr_el1(self.spsr_el1);
            asm::set_cntkctl_el1(self.cntkctl_el1);
            asm::set_cntv_ctl_el0(self.cntv_ctl_el0);
            asm::set_cntv_cval_el0(self.cntv_cval_el0);
        }
    }
}

/// The EL1 register state a pCPU has before any guest has ever run on it
/// (i.e. whatever the firmware/bootloader left behind at EL2 entry).
/// Captured once, the first time `create_vm` runs on a given pCPU, and
/// reused as the initial `El1Context` for every VCPU ever created on that
/// pCPU afterwards (see `VM::set_initial_context`): a freshly queued VCPU
/// that has never run yet must start from this same pristine state, not
/// from whatever another, already-running VCPU has since programmed into
/// these registers.
static RESET_EL1_CONTEXT: Mutex<Option<El1Context>> = Mutex::new(None);

fn reset_el1_context() -> El1Context {
    *RESET_EL1_CONTEXT
        .lock()
        .get_or_insert_with(El1Context::capture_current)
}

/// Saved off-CPU state of a VCPU that is not currently the one physically
/// executing on its owning pCPU. Populated with the VCPU's initial boot
/// state (kernel entry point / DTB pointer argument) at creation time, and
/// overwritten with a live snapshot every time this VCPU cooperatively
/// yields the pCPU (see `try_yield_to_next_vcpu`).
///
/// Covers what is needed to correctly resume a guest that was interrupted
/// at a trap boundary, now that each VCPU has its own Stage 2 translation
/// table and VMID (see `VM::stage2_table_address`/`VM::vmid` and
/// `paging::activate_stage2_translation_table`): the GPRs captured in the
/// trap frame, ELR_EL2/SPSR_EL2 (where/how to resume), the hardware GICH
/// List Registers (pending/active virtual interrupt state), since those
/// are physical-GIC-banked-per-pCPU resources shared by every VCPU
/// scheduled on the same pCPU, and every EL0/EL1 system register banked
/// per Exception Level rather than per-VM (see `El1Context`).
pub struct VcpuContext {
    regs: [u64; Registers::NUMBER_OF_SLOTS],
    elr_el2: u64,
    spsr_el2: u64,
    gich_lr: [u32; vgic::NUMBER_OF_SUPPORTED_LRS],
    el1: El1Context,
}

impl VcpuContext {
    fn new() -> Self {
        Self {
            regs: [0; Registers::NUMBER_OF_SLOTS],
            elr_el2: 0,
            spsr_el2: 0,
            gich_lr: [0; vgic::NUMBER_OF_SUPPORTED_LRS],
            el1: reset_el1_context(),
        }
    }
}

pub struct VM {
    vm_id: usize,
    /// MPIDR-derived affinity of the physical CPU this VCPU is scheduled
    /// on. Used to find other runnable VCPUs sharing the same pCPU (see
    /// `try_yield_to_next_vcpu`); VCPUs on different pCPUs are never
    /// switched between each other since there is only one pCPU per VM
    /// physically executing at any given moment.
    owner_affinity: u64,
    /// Physical base address of this VCPU's own, private Stage 2
    /// translation table (see `paging::create_stage2_translation_table`).
    /// Every VCPU gets its own table (even when queued behind another VCPU
    /// on the same pCPU) so that switching between them can never expose
    /// one VCPU's guest-physical RAM through another's stale mappings.
    stage2_table_address: usize,
    /// This VCPU's unique VTTBR_EL2 VMID (see `registers::VTTBR_VMID`),
    /// tagging its Stage 2 (and combined Stage 1+2) TLB entries so
    /// `paging::activate_stage2_translation_table` never needs to flush the
    /// TLB on every cooperative VCPU switch.
    vmid: u64,
    ram_virtual_base_address: usize,
    ram_physical_base_address: usize,
    ram_size: usize,
    mmio_handlers: LinkedList<MmioEntry>,
    gic_distributor_mmio: Arc<Mutex<GicDistributorMmio>>,
    pl011_mmio: Arc<Mutex<Pl011Mmio>>,
    virtio_net_mmio: Arc<Mutex<VirtioNetMmio>>,
    context: Mutex<VcpuContext>,
}

#[repr(C)]
struct KernelHeader {
    code0: u32,
    code1: u32,
    text_offset: u64,
    image_size: u64,
    flags: u64,
    res2: u64,
    res3: u64,
    res4: u64,
    magic: u32,
    res5: u32,
}

static VM_LIST: Mutex<LinkedList<Arc<VM>>> = Mutex::new(LinkedList::new());
static NEXT_VM_ID: AtomicUsize = AtomicUsize::new(0);
static ACTIVE_VM: Mutex<Option<Arc<VM>>> = Mutex::new(None);
/// Allocates unique VTTBR_EL2 VMIDs (see `registers::VTTBR_VMID`). The
/// field is 8 bits wide, so this wraps modulo 256; with one hypervisor
/// instance managing at most a handful of VCPUs across up to 4 pCPUs (on
/// Raspberry Pi 4), collisions are not a practical concern.
static NEXT_VMID: AtomicUsize = AtomicUsize::new(0);

fn allocate_vmid() -> u64 {
    (NEXT_VMID.fetch_add(1, Ordering::Relaxed) & 0xFF) as u64
}

impl VM {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        vm_id: usize,
        owner_affinity: u64,
        stage2_table_address: usize,
        vmid: u64,
        ram_virtual_base_address: usize,
        ram_physical_base_address: usize,
        ram_size: usize,
        mmio_handlers: LinkedList<MmioEntry>,
        gic_distributor_mmio: Arc<Mutex<GicDistributorMmio>>,
        pl011_mmio: Arc<Mutex<Pl011Mmio>>,
        virtio_net_mmio: Arc<Mutex<VirtioNetMmio>>,
    ) -> Self {
        Self {
            vm_id,
            owner_affinity,
            stage2_table_address,
            vmid,
            ram_virtual_base_address,
            ram_physical_base_address,
            ram_size,
            mmio_handlers,
            gic_distributor_mmio,
            pl011_mmio,
            virtio_net_mmio,
            context: Mutex::new(VcpuContext::new()),
        }
    }

    /// Populates this (not-yet-run) VCPU's saved context with its initial
    /// boot state: `boot_argument` (the DTB pointer Linux's entry point
    /// expects in x0) and `boot_entry_point` (ELR_EL2 to resume at, in
    /// EL1h per `SPSR_EL2_M_EL1H`). Must be called exactly once, before
    /// this VCPU is ever activated/restored, since `create_vm` only knows
    /// the true kernel entry point once the image has been loaded and its
    /// header parsed.
    fn set_initial_context(&self, boot_entry_point: usize, boot_argument: usize) {
        let mut context = self.context.lock();
        context.regs[0] = boot_argument as u64;
        context.elr_el2 = boot_entry_point as u64;
        context.spsr_el2 = SPSR_EL2_M_EL1H;
    }

    /// Saves this VCPU's live state (GPRs from the current trap frame,
    /// ELR_EL2/SPSR_EL2 telling it where/how to resume, the physical GICH
    /// List Registers holding its virtual interrupt state, and every
    /// EL0/EL1 system register banked per Exception Level -- see
    /// `El1Context`) so it can be resumed later by `restore_context` on
    /// the same pCPU.
    fn save_context(&self, registers: &Registers, elr_el2: u64, spsr_el2: u64) {
        let mut context = self.context.lock();
        context.regs = registers.as_array();
        context.elr_el2 = elr_el2;
        context.spsr_el2 = spsr_el2;
        for i in 0..vgic::NUMBER_OF_SUPPORTED_LRS {
            context.gich_lr[i] = physical_gicv2::get_gich_lr(i);
        }
        context.el1 = El1Context::capture_current();
    }

    /// Restores this VCPU's previously saved state into the current trap
    /// frame, the physical GICH List Registers, and every EL0/EL1 system
    /// register banked per Exception Level (see `El1Context`), and returns
    /// the (ELR_EL2, SPSR_EL2) pair the caller must program before
    /// `eret`ing so execution resumes exactly where this VCPU left off.
    fn restore_context(&self, registers: &mut Registers) -> (u64, u64) {
        let context = self.context.lock();
        registers.load_array(&context.regs);
        for i in 0..vgic::NUMBER_OF_SUPPORTED_LRS {
            physical_gicv2::set_gich_lr(i, context.gich_lr[i]);
        }
        context.el1.activate();
        (context.elr_el2, context.spsr_el2)
    }

    /// This VM's unique identifier (see `create_vm`/`NEXT_VM_ID`), stored
    /// per-pCPU in TPIDR_EL2 by `vm::get_current_vm`'s callers so it can be
    /// looked up again from `VM_LIST`.
    pub fn vm_id(&self) -> usize {
        self.vm_id
    }

    pub fn handle_mmio_read(&self, address: usize, access_width: u64) -> Result<u64, ()> {
        for e in &self.mmio_handlers {
            if e.base_address <= address && address < (e.base_address + e.length) {
                return e
                    .handler
                    .lock()
                    .read(address - e.base_address, access_width);
            }
        }
        Err(())
    }

    pub fn handle_mmio_write(
        &self,
        address: usize,
        access_width: u64,
        value: u64,
    ) -> Result<(), ()> {
        for e in &self.mmio_handlers {
            if e.base_address <= address && address < (e.base_address + e.length) {
                return e
                    .handler
                    .lock()
                    .write(address - e.base_address, access_width, value);
            }
        }
        Err(())
    }

    pub fn get_physical_address(&self, virtual_address: usize) -> Option<usize> {
        if (self.ram_virtual_base_address..(self.ram_virtual_base_address + self.ram_size))
            .contains(&virtual_address)
        {
            Some(virtual_address - self.ram_virtual_base_address + self.ram_physical_base_address)
        } else {
            None
        }
    }

    pub fn get_gic_distributor_mmio(&self) -> &Mutex<GicDistributorMmio> {
        &self.gic_distributor_mmio
    }

    pub fn get_pl011_mmio(&self) -> &Mutex<Pl011Mmio> {
        &self.pl011_mmio
    }

    pub fn get_virtio_net_mmio(&self) -> &Mutex<VirtioNetMmio> {
        &self.virtio_net_mmio
    }
}

impl MmioEntry {
    pub fn new(
        base_address: usize,
        length: usize,
        handler: Arc<Mutex<dyn MmioHandler + Send>>,
    ) -> Self {
        Self {
            base_address,
            length,
            handler,
        }
    }
}

pub fn create_vm(
    fat32: &Fat32,
    blk: &mut dyn BlockDevice,
    gic_distributor: &GicDistributor,
    gic_hypervisor_interface: &GicHypervisorInterface,
    gic_virtual_cpu_interface_physical_address: usize,
    gic_virtual_cpu_interface_size: usize,
    net_mac: [u8; 6],
) -> (usize, usize) {
    const RAM_VIRTUAL_BASE: usize = 0x40000000;
    /// RAM SIZE: 256MiB
    const RAM_SIZE: usize = 0x10000000;
    const ALIGN_SIZE: usize = 0x200000;
    /* Address of the GICv2 CPU Interface (GICC) shown to the guest
     * (must match reg[1] of the intc node in scripts/virt.dts) */
    const GUEST_GIC_CPU_INTERFACE_ADDRESS: usize = 0x8010000;

    /* Set up the basic elements of the virtual machine */
    let ram_physical_address = crate::allocate_pages(RAM_SIZE >> PAGE_SHIFT, PAGE_SHIFT)
        .expect("Failed to allocate memory for VM.");
    let vm_id = NEXT_VM_ID.fetch_add(1, Ordering::Relaxed);
    let cpu_mpidr = asm::get_mpidr_el1();

    /* Diagnostic (temporary): check whether the just-allocated guest RAM
     * physical range overlaps the host-owned FAT32 root-directory buffer.
     * If it does, the guest zeroing/using its own RAM during early boot
     * would silently corrupt that buffer (and anything else backed by the
     * same physical pages), which would explain a virtio-blk 0-capacity
     * bug observed after VM creation but before the guest's first MMIO
     * access to the device. */
    let (fat_buf_start, fat_buf_end) = fat32.debug_buffer_range();
    let ram_end = ram_physical_address + RAM_SIZE;
    println!(
        "Guest RAM: [{:#X} ~ {:#X}], FAT32 root dir buffer: [{:#X} ~ {:#X}]{}",
        ram_physical_address,
        ram_end,
        fat_buf_start,
        fat_buf_end,
        if fat_buf_start < ram_end && fat_buf_end > ram_physical_address {
            " <-- OVERLAP!"
        } else {
            ""
        }
    );

    /* Configure hardware related to virtualization */
    /* Set up registers */
    setup_hypervisor_registers();

    /* Every VCPU gets its own private Stage 2 translation table (and a
     * unique VMID), rather than reusing whatever table happens to be live
     * in VTTBR_EL2. This is what makes it safe to `create_vm` a VCPU that
     * is only queued behind another one already running on this pCPU (see
     * `is_first_on_this_pcpu` below): building up this table's mappings
     * can never disturb the currently active VCPU's own Stage 2 mappings,
     * since they live in a different table entirely. */
    let stage2_table_address = create_stage2_translation_table();
    let vmid = allocate_vmid();
    map_address_stage2(
        stage2_table_address,
        ram_physical_address,
        RAM_VIRTUAL_BASE,
        RAM_SIZE,
        true,
        true,
    )
    .expect("Failed to map memory");

    /* Directly passthrough-map the GICv2 Virtual CPU Interface (GICV) to the guest's GICC address.
     * (The guest accesses the hardware virtual CPU interface directly, so EOI/ACK do not trap) */
    map_device_stage2(
        stage2_table_address,
        gic_virtual_cpu_interface_physical_address,
        GUEST_GIC_CPU_INTERFACE_ADDRESS,
        gic_virtual_cpu_interface_size,
        true,
        true,
    )
    .expect("Failed to map GICv2 Virtual CPU Interface");

    /* Initialize the virtual GIC */
    vgic::init_vgic(gic_hypervisor_interface, gic_distributor);

    /* Initialize the Generic Timer */
    generic_timer::init_generic_timer_local(gic_distributor);

    /* Initialize the MMIO handlers */
    let mut mmio_handlers = LinkedList::new();

    /* PL011 */
    let pl011_mmio = Arc::new(Mutex::new(Pl011Mmio::new()));
    mmio_handlers.push_back(MmioEntry::new(0x9000000, 0x1000, pl011_mmio.clone()));

    /* Virtio-Blk */
    let file_name = [b'D', b'I', b'S', b'K', b'0' + vm_id as u8];
    let disk_file = fat32
        .search_file(core::str::from_utf8(&file_name).unwrap())
        .expect("Failed to find Disk");
    /* Diagnostic (temporary): on real Raspberry Pi 4 hardware, virtio_blk in
     * the guest has intermittently reported a 0-block capacity despite
     * Fat32::list_files() printing the correct size for the same file
     * moments earlier from the same in-memory root directory buffer. Print
     * the size captured here again to narrow down whether the corruption
     * (if any) happens before or after this point. */
    println!(
        "DISK{vm_id} file size at VM creation: {:#X}",
        disk_file.get_file_size()
    );
    let virtio_blk_mmio = Arc::new(Mutex::new(VirtioBlkMmio::new(disk_file)));
    /* Diagnostic (temporary): check whether the Arc<Mutex<VirtioBlkMmio>>
     * heap allocation itself (which holds the FileInfo/file_size by value)
     * falls inside the guest RAM physical range. If it does, anything the
     * guest writes to its own low RAM during early boot (BSS clear, page
     * tables, etc.) would silently corrupt this host-owned struct. */
    let virtio_blk_mmio_addr = Arc::as_ptr(&virtio_blk_mmio) as usize;
    println!(
        "VirtioBlkMmio heap address: {:#X} (Guest RAM: [{:#X} ~ {:#X}]){}",
        virtio_blk_mmio_addr,
        ram_physical_address,
        ram_end,
        if virtio_blk_mmio_addr >= ram_physical_address && virtio_blk_mmio_addr < ram_end {
            " <-- OVERLAP!"
        } else {
            ""
        }
    );
    mmio_handlers.push_back(MmioEntry::new(0xa000000, 0x0200, virtio_blk_mmio));

    /* GIC Distributor(Virtual) */
    let gic_distributor_mmio = Arc::new(Mutex::new(GicDistributorMmio::new(
        gic_distributor.get_own_target(),
    )));
    mmio_handlers.push_back(MmioEntry::new(
        0x8000000,
        GicDistributorMmio::MMIO_SIZE,
        gic_distributor_mmio.clone(),
    ));

    /* Virtio-Net */
    let virtio_net_mmio = Arc::new(Mutex::new(VirtioNetMmio::new(net_mac)));
    mmio_handlers.push_back(MmioEntry::new(0xa000200, 0x0200, virtio_net_mmio.clone()));

    /* Create the VM structure */
    let owner_affinity = asm::mpidr_to_affinity(cpu_mpidr);
    let vm = VM::new(
        vm_id,
        owner_affinity,
        stage2_table_address,
        vmid,
        RAM_VIRTUAL_BASE,
        ram_physical_address,
        RAM_SIZE,
        mmio_handlers,
        gic_distributor_mmio,
        pl011_mmio,
        virtio_net_mmio,
    );

    /* Load the Linux kernel and devicetree */
    let kernel = fat32.search_file("IMAGE").unwrap();
    let dtb = fat32.search_file("DTB").unwrap();
    let dtb_size = dtb.get_file_size();
    let kernel_size = kernel.get_file_size();
    let kernel_virtual_address =
        ((RAM_VIRTUAL_BASE + dtb_size - 1) & !(ALIGN_SIZE - 1)) + ALIGN_SIZE;
    let kernel_physical_address = vm.get_physical_address(kernel_virtual_address).unwrap();

    fat32
        .read(&dtb, blk, ram_physical_address, 0, dtb_size)
        .expect("Failed to read DTB");
    fat32
        .read(&kernel, blk, kernel_physical_address, 0, kernel_size)
        .expect("Failed to read Kernel");

    /* Make the freshly loaded DTB and kernel image visible to the
     * instruction fetch path. Without this, a CPU with real (non-modeled)
     * caches may execute stale/garbage instructions instead of the code
     * just written by the SDHCI/FAT32 read above (this is not observable
     * under QEMU, which does not model cache incoherency). */
    unsafe {
        asm::clean_dcache_and_invalidate_icache(ram_physical_address, dtb_size);
        asm::clean_dcache_and_invalidate_icache(kernel_physical_address, kernel_size);
    };

    /* Parse the Linux kernel header */
    let header = unsafe { &*(kernel_physical_address as *const KernelHeader) };
    if header.magic != 0x644D5241 {
        panic!("Invalid Kernel Magic: {:#X}", header.magic);
    }
    let mut text_offset = header.text_offset;
    let image_size = header.image_size;
    if image_size == 0 {
        text_offset = 0x80000;
    }

    /* Add to the VM structure list */
    let boot_entry_point = kernel_virtual_address + text_offset as usize;
    let boot_argument = RAM_VIRTUAL_BASE;
    vm.set_initial_context(boot_entry_point, boot_argument);
    let vm = Arc::new(vm);
    /* If another VCPU is already scheduled on this same pCPU, this one is
     * only *queued*: leave the currently running VCPU active and let the
     * cooperative scheduler (`try_yield_to_next_vcpu`, triggered on the
     * active VCPU's next guest WFI/WFE trap) pick this one up from its
     * saved (boot) context. Otherwise, this is the first/only VCPU for
     * this pCPU, so the caller is expected to `boot_vm()` it immediately. */
    let is_first_on_this_pcpu = !VM_LIST
        .lock()
        .iter()
        .any(|v| v.owner_affinity == owner_affinity);
    VM_LIST.lock().push_back(vm.clone());
    if is_first_on_this_pcpu {
        switch_active_vm(vm_id);
        unsafe { asm::set_tpidr_el2(vm_id as u64) };
        /* This is the only VCPU on this pCPU so far, so its Stage 2 table
         * becomes the live one immediately; the caller (`boot_vm`) erets
         * into it right after this returns. */
        activate_stage2_translation_table(stage2_table_address, vmid);
        println!("Created VM{vm_id} on the CPU(MPIDR_EL1: {:#X})", cpu_mpidr);
    } else {
        println!(
            "Created VM{vm_id} on the CPU(MPIDR_EL1: {:#X}), queued behind VM{} \
             (will run once the active VCPU on this pCPU yields)",
            cpu_mpidr,
            get_current_vm().vm_id
        );
    }

    (boot_entry_point, boot_argument)
}

pub fn boot_vm(entry_point: usize, argument: usize) -> ! {
    unsafe {
        /* Boot the virtual machine */
        asm::set_spsr_el2(SPSR_EL2_M_EL1H);
        asm::set_elr_el2(entry_point as u64);
        asm::eret(argument as u64, 0, 0, 0);
    }
}

fn setup_hypervisor_registers() {
    /* MIDR_EL1 */
    unsafe { asm::set_vpidr_el2(asm::get_midr_el1()) };

    /* MPIDR_EL1 */
    unsafe { asm::set_vmpidr_el2(asm::get_mpidr_el1()) };

    /* HCR_EL2 */
    let hcr_el2 = HCR_EL2_RW
        | HCR_EL2_API
        | HCR_EL2_AMO
        | HCR_EL2_IMO
        | HCR_EL2_FMO
        | HCR_EL2_TWI
        | HCR_EL2_TSC
        | HCR_EL2_VM;
    unsafe { asm::set_hcr_el2(hcr_el2) };
}

/// Makes an already-created VM (see [`create_vm`]) the active guest on the
/// *current* physical CPU, without creating any new VM, Stage 2 table, or
/// VMID. Used when a guest's own (virtualized) PSCI CPU_ON brings up an
/// additional vCPU of a VM that already has other vCPUs concurrently
/// running on other pCPUs -- true SMP -- as opposed to [`create_vm`]'s "one
/// brand-new, independent VM per pCPU" model used by the `boot`/`spawn`
/// debug console commands.
///
/// The EL2 system registers this configures (HCR_EL2, VMPIDR_EL2, etc.) are
/// banked per physical CPU by hardware, so, unlike [`create_vm`], this must
/// run once on *every* pCPU that will execute a vCPU of `vm`, not just once
/// per VM.
pub fn activate_vm_on_this_pcpu(vm: &Arc<VM>) {
    setup_hypervisor_registers();
    set_vtcr_el2_for_this_pcpu();
    activate_stage2_translation_table(vm.stage2_table_address, vm.vmid);
    unsafe { asm::set_tpidr_el2(vm.vm_id as u64) };
}

pub fn input_uart(c: u8) {
    let vm = get_active_vm();
    vm.get_pl011_mmio()
        .lock()
        .push(c, &mut vm.get_gic_distributor_mmio().lock());
}

/// Injects a received Ethernet frame into the current VM's Virtio-Net device.
/// Called from the physical Virtio-Net interrupt handler.
pub fn input_net_packet(data: &[u8]) {
    get_current_vm().get_virtio_net_mmio().lock().push_rx(data);
}

pub fn get_current_vm() -> Arc<VM> {
    let vm_id = asm::get_tpidr_el2() as usize;
    VM_LIST
        .lock()
        .iter()
        .find(|vm| vm.vm_id == vm_id)
        .unwrap()
        .clone()
}

pub fn get_active_vm() -> Arc<VM> {
    ACTIVE_VM.lock().clone().unwrap()
}

pub fn switch_active_vm(vm_id: usize) -> bool {
    if let Some(vm) = VM_LIST.lock().iter_mut().find(|vm| vm.vm_id == vm_id) {
        *ACTIVE_VM.lock() = Some(vm.clone());
        true
    } else {
        false
    }
}

/// Finds the VCPU that should run next on the *current* pCPU after
/// `current_vm_id` yields, by round-robining among every VM whose
/// `owner_affinity` matches this pCPU (i.e. every VCPU ever created on it,
/// via the initial `create_vm` call or a later queued one). Returns `None`
/// if `current_vm_id` is the only VCPU on this pCPU (nothing to switch to).
fn find_next_same_affinity_vcpu(current_vm_id: usize, affinity: u64) -> Option<Arc<VM>> {
    let candidates: Vec<Arc<VM>> = VM_LIST
        .lock()
        .iter()
        .filter(|vm| vm.owner_affinity == affinity)
        .cloned()
        .collect();
    if candidates.len() <= 1 {
        return None;
    }
    let position = candidates.iter().position(|vm| vm.vm_id == current_vm_id)?;
    Some(candidates[(position + 1) % candidates.len()].clone())
}

/// Cooperatively switches the current pCPU from the currently running VCPU
/// to the next runnable VCPU sharing the same pCPU, if any (round-robin).
/// Called from the guest WFI/WFE trap handler (`exception::wfx_handler`),
/// since that is the only point this hypervisor currently has a reliable,
/// architecturally-defined opportunity to regain control from a running
/// guest without an asynchronous preemption timer (added in a later phase).
///
/// `registers` is the current trap frame: on a successful switch it is
/// overwritten in place with the target VCPU's saved GPRs, and this
/// function also reprograms ELR_EL2/SPSR_EL2 directly, so that when
/// `synchronous_handler`'s caller falls through to the shared
/// `exit_exception` epilogue, it `eret`s into the *target* VCPU instead of
/// resuming the one that actually took the trap.
///
/// Returns `true` if a switch was performed (the caller must NOT also
/// advance the outgoing VCPU's ELR_EL2, since this function already does
/// so before saving its context), or `false` if there was nothing else to
/// switch to (the caller should fall back to simply advancing past the
/// WFI/WFE as before).
///
/// Note: each VCPU now has its own private Stage 2 translation table and
/// VMID (see `VM::stage2_table_address`/`VM::vmid`, populated in
/// `create_vm` and activated here via
/// `paging::activate_stage2_translation_table`), so multiple VCPUs queued
/// on the same pCPU no longer alias each other's guest-physical RAM at
/// 0x40000000+. Preemptive scheduling (an asynchronous timer instead of
/// only switching at a cooperative WFI/WFE trap) remains for a later
/// phase.
pub fn try_yield_to_next_vcpu(registers: &mut Registers) -> bool {
    let affinity = asm::mpidr_to_affinity(asm::get_mpidr_el1());
    let current = get_current_vm();
    let Some(next) = find_next_same_affinity_vcpu(current.vm_id, affinity) else {
        return false;
    };

    /* Advance past the WFI/WFE *before* snapshotting ELR_EL2, so that when
     * this (outgoing) VCPU is eventually restored, it resumes just after
     * the instruction that yielded rather than re-trapping on it forever. */
    unsafe { asm::advance_elr_el2() };
    let elr_el2 = asm::get_elr_el2();
    let spsr_el2 = asm::get_spsr_el2();
    current.save_context(registers, elr_el2, spsr_el2);

    let (next_elr_el2, next_spsr_el2) = next.restore_context(registers);
    /* Make the target VCPU's own Stage 2 mappings live before resuming it;
     * its unique VMID means this never requires a TLB flush (see
     * `paging::activate_stage2_translation_table`), unlike a naive
     * shared-table approach would. */
    activate_stage2_translation_table(next.stage2_table_address, next.vmid);
    unsafe {
        asm::set_elr_el2(next_elr_el2);
        asm::set_spsr_el2(next_spsr_el2);
        asm::set_tpidr_el2(next.vm_id as u64);
    }
    *ACTIVE_VM.lock() = Some(next.clone());
    true
}
