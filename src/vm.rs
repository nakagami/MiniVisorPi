//!
//! Virtual Machine management module
//!

use crate::asm;
use crate::drivers::block_device::BlockDevice;
use crate::drivers::{
    generic_timer,
    gicv2::{GicDistributor, GicHypervisorInterface},
};
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

pub trait MmioHandler {
    fn read(&mut self, offset: usize, access_width: u64) -> Result<u64, ()>;
    fn write(&mut self, offset: usize, access_width: u64, value: u64) -> Result<(), ()>;
}

pub struct MmioEntry {
    base_address: usize,
    length: usize,
    handler: Arc<Mutex<dyn MmioHandler + Send>>,
}

pub struct VM {
    vm_id: usize,
    ram_virtual_base_address: usize,
    ram_physical_base_address: usize,
    ram_size: usize,
    mmio_handlers: LinkedList<MmioEntry>,
    gic_distributor_mmio: Arc<Mutex<GicDistributorMmio>>,
    pl011_mmio: Arc<Mutex<Pl011Mmio>>,
    virtio_net_mmio: Arc<Mutex<VirtioNetMmio>>,
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

impl VM {
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        vm_id: usize,
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
            ram_virtual_base_address,
            ram_physical_base_address,
            ram_size,
            mmio_handlers,
            gic_distributor_mmio,
            pl011_mmio,
            virtio_net_mmio,
        }
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

    /* Configure hardware related to virtualization */
    /* Set up registers */
    setup_hypervisor_registers();

    /* Initialize the Stage 2 translation table */
    init_stage2_translation_table();
    map_address_stage2(ram_physical_address, RAM_VIRTUAL_BASE, RAM_SIZE, true, true)
        .expect("Failed to map memory");

    /* Directly passthrough-map the GICv2 Virtual CPU Interface (GICV) to the guest's GICC address.
     * (The guest accesses the hardware virtual CPU interface directly, so EOI/ACK do not trap) */
    map_device_stage2(
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
    mmio_handlers.push_back(MmioEntry::new(
        0xa000000,
        0x0200,
        Arc::new(Mutex::new(VirtioBlkMmio::new(disk_file))),
    ));

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
    let vm = VM::new(
        vm_id,
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
    VM_LIST.lock().push_back(Arc::new(vm));
    switch_active_vm(vm_id);

    unsafe { asm::set_tpidr_el2(vm_id as u64) };
    println!("Created VM{vm_id} on the CPU(MPIDR_EL1: {:#X})", cpu_mpidr);

    (
        kernel_virtual_address + text_offset as usize,
        RAM_VIRTUAL_BASE,
    )
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
    let hcr_el2 = HCR_EL2_RW | HCR_EL2_API | HCR_EL2_AMO | HCR_EL2_IMO | HCR_EL2_FMO | HCR_EL2_VM;
    unsafe { asm::set_hcr_el2(hcr_el2) };
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
