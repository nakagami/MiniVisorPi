#![no_std]
#![no_main]

extern crate alloc;

#[macro_use]
mod serial;
mod asm;
mod block_backend;
mod console;
mod dtb;
mod drivers {
    pub mod block_device;
    pub mod generic_timer;
    pub mod genet;
    pub mod gicv2;
    pub mod pcie_brcm;
    pub mod pl011;
    pub mod sdhci;
    pub mod usb_mass_storage;
    pub mod virtio;
    pub mod xhci;
    pub mod virtio_blk;
    pub mod virtio_net;
}
mod elf;
mod exception;
mod fat32;
mod guest_memory;
mod lock;
mod memory_allocator;
mod mmio {
    pub mod gicv2;
    pub mod pl011;
    pub mod virtio_blk;
    pub mod virtio_net;
}
mod paging;
mod psci;
mod registers;
mod vgic;
mod vgic_lr;
mod vm;

use block_backend::BlockBackend;
use drivers::{generic_timer, genet, gicv2, pcie_brcm, pl011, usb_mass_storage, virtio_blk, virtio_net, xhci};
use lock::Mutex;
use psci::PsciErrorCodes;
use serial::SerialDevice;

use core::alloc::{GlobalAlloc, Layout};
use core::arch::naked_asm;
use core::ffi::CStr;
use core::mem::MaybeUninit;
use core::slice;
use core::sync::atomic::{AtomicBool, Ordering};

use alloc::collections::linked_list::LinkedList;
use alloc::sync::Arc;
use alloc::vec::Vec;

struct GlobalAllocator {}

enum PhysicalNet {
    Virtio(virtio_net::VirtioNet),
    Genet(genet::Genet),
}

impl PhysicalNet {
    fn get_mac_address(&self) -> [u8; 6] {
        match self {
            Self::Virtio(net) => net.get_mac_address(),
            Self::Genet(net) => net.get_mac_address(),
        }
    }

    fn send(&mut self, buffer_address: usize, length: usize) -> Result<(), ()> {
        match self {
            Self::Virtio(net) => net.send(buffer_address, length),
            Self::Genet(net) => net.send(buffer_address, length),
        }
    }

    fn poll_rx(&mut self, buffer: &mut [u8]) -> Option<usize> {
        match self {
            Self::Virtio(net) => net.poll_rx(buffer),
            Self::Genet(net) => net.poll_rx(buffer),
        }
    }

    fn requires_wfx_polling(&self) -> bool {
        matches!(self, Self::Genet(_))
    }
}

/// Global variable storage
static PL011_DEVICE: Mutex<pl011::Pl011> = Mutex::new(pl011::Pl011::invalid());
static mut PL011_INT_ID: u32 = 0;
static MEMORY_ALLOCATOR: Mutex<memory_allocator::MemoryAllocator> =
    Mutex::new(memory_allocator::MemoryAllocator::new());
static VIRTIO_BLK: Mutex<BlockBackend> = Mutex::new(BlockBackend::invalid());
static PHYSICAL_NET: Mutex<Option<PhysicalNet>> = Mutex::new(None);
static mut VIRTIO_NET_INT_ID: u32 = 0;
static mut FAT32: MaybeUninit<fat32::Fat32> = MaybeUninit::uninit();
#[global_allocator]
static GLOBAL_ALLOCATOR: GlobalAllocator = GlobalAllocator {};
static CONSOLE: Mutex<console::Console> = Mutex::new(console::Console::new());
static IS_CONSOLE_ACTIVE: AtomicBool = AtomicBool::new(false);
/// Whether the machine's DTB advertises EL3 PSCI firmware (and `smc` is
/// therefore safe to execute at EL2). False e.g. on the Raspberry Pi 4's
/// stock firmware, where secondary CPUs are brought up via the ARM
/// spin-table protocol instead and `smc` is UNDEFINED. Read by
/// [`handle_guest_smc`] to decide between forwarding a guest PSCI call to
/// real firmware and emulating it.
static HAS_PSCI: AtomicBool = AtomicBool::new(false);
static mut DTB: MaybeUninit<dtb::Dtb> = MaybeUninit::uninit();

/// A physical CPU parked (WFE-looping) at EL2 by
/// [`park_secondary_cpus_for_smp`], available to be handed a vCPU of an
/// existing VM by a guest-issued (virtualized) PSCI CPU_ON call (see
/// [`handle_guest_cpu_on`]). This is what lets a single guest kernel image
/// use multiple CPUs (true SMP), instead of every physical CPU always
/// hosting its own brand-new, independent VM (the `boot`/`spawn` debug
/// console commands' model).
struct ParkedCpu {
    affinity: u64,
    launch: Mutex<Option<SmpLaunchRequest>>,
}

/// What a guest-issued PSCI CPU_ON asked a parked physical CPU to run:
/// which VM to join (see `vm::activate_vm_on_this_pcpu`), and the
/// entry point/context id the guest itself supplied.
struct SmpLaunchRequest {
    vm: Arc<vm::VM>,
    entry_point: usize,
    context_id: usize,
}

static PARKED_CPUS: Mutex<LinkedList<Arc<ParkedCpu>>> = Mutex::new(LinkedList::new());

/// Per-pCPU stack top addresses (as originally handed to
/// [`asm::smp_park_entry`]/[`psci::spin_table_cpu_on`] by
/// [`park_secondary_cpus_for_smp`]), keyed by that pCPU's MPIDR-derived
/// affinity. Looked back up by [`handle_guest_cpu_off`] to cleanly re-park
/// a physical CPU (via [`asm::reset_stack_and_park`]) after a guest PSCI
/// CPU_OFF retires its last VCPU, without leaking stack across repeated
/// CPU_OFF/CPU_ON cycles.
static STACK_TOPS: Mutex<Vec<(u64, u64)>> = Mutex::new(Vec::new());


/// Constants
const STACK_SIZE: usize = 0x10000;
const CONSOLE_SWITCH_KEY: u8 = 0x13; /* Ctrl + S */

/// The real ELF entry point named by `ENTRY(main)` in scripts/{pi4,qemu}.ld.
///
/// This must be a bare trampoline with NO Rust-generated prologue: U-Boot (or
/// QEMU) jumps here with the stack pointer set to the actual top of the
/// STACK_SIZE region this hypervisor is entitled to use, but a normal `extern
/// "C" fn main(...)` would first run its own prologue (`sub sp, sp, #N` to
/// make room for its own locals) *before* any Rust statement -- including one
/// that tries to read `sp` to learn where the stack starts -- ever executes.
/// That previously made `setup_memory()`'s stack reservation undercount the
/// true stack region by `main()`'s own frame size: whichever of `main()`'s
/// locals happened to sit above the (already-too-low) sampled `sp` value was
/// left unreserved and thus available to the general-purpose page allocator,
/// which then handed that same memory to the xHCI driver as its Device
/// Context Base Address Array -- and DmaRegion::new() zeroes every page it
/// allocates, silently clobbering those "unprotected" locals (including, on
/// real Raspberry Pi 4 hardware booting from a USB3/xHCI mass-storage
/// device, the on-stack `Dtb`'s `header` pointer, producing the "null
/// pointer dereference occurred" panic in dtb.rs). Capturing the pristine
/// entry `sp` here, before any frame is carved out of it, and threading it
/// through to `setup_memory()` fixes this at the source.
#[unsafe(naked)]
#[unsafe(no_mangle)]
pub extern "C" fn main() -> ! {
    naked_asm!(
        "mov x2, sp",
        "b {entry_main}",
        entry_main = sym entry_main,
    )
}

extern "C" fn entry_main(argc: usize, argv: *const *const u8, entry_stack_pointer: usize) -> usize {
    if argc != 2 {
        return 1;
    }
    let args = unsafe { slice::from_raw_parts(argv, argc) };
    /* argv[0] is the DTB */
    let Ok(arg_0) = unsafe { CStr::from_ptr(args[0]) }.to_str() else {
        /* Conversion failed */
        return 2;
    };
    let Some(dtb_address) = str_to_usize(arg_0) else {
        return 3;
    };
    let Ok(dtb) = dtb::Dtb::new(dtb_address) else {
        return 4;
    };
    if let Err(e) = init_serial_port(&dtb) {
        return e;
    }

    println!("Hello, world!");

    let current_el = asm::get_currentel() >> 2;
    println!("CurrentEL: {}", current_el);
    assert_eq!(current_el, 2);

    /* Set up memory management */
    /* argv[1] is the ELF header location */
    let arg_1 = unsafe { CStr::from_ptr(args[1]) }
        .to_str()
        .expect("Failed to get argv[1]");
    let elf_address = str_to_usize(arg_1).expect("Failed to convert the address");
    setup_memory(&dtb, dtb_address, elf_address, entry_stack_pointer);

    exception::setup_exception();
    /* Mask IRQ/FIQ for the remainder of this hypervisor's own EL2 execution context,
     * before any physical interrupt source (e.g. the PL011's) gets enabled below. This
     * crate's Mutex (src/lock.rs) always restores whichever DAIF state it observed on
     * lock(), so setting it once here keeps IRQ/FIQ masked at EL2 permanently, even
     * across future lock()/unlock() cycles.
     *
     * This matters because handling a physical interrupt here (e.g. crate::handle_input
     * -> vm::input_uart -> vm::get_active_vm()) requires a VM to already exist and be
     * marked active, but no VM exists yet this early in boot (Created VM0 doesn't print
     * until much later). If a physical interrupt fires before then while unmasked --
     * confirmed to happen on real Raspberry Pi 4 hardware, apparently from electrical
     * noise on the UART RX line during power-up, well before any user keypress -- it
     * previously caused an untimely handle_input() call that corrupted VM/device state
     * or crashed outright (e.g. Virtio-Blk reporting 0 blocks / a stray "Failed to
     * handle MMIO" panic during early guest boot).
     *
     * This does not prevent the guest from receiving physical interrupts once it's
     * actually running: HCR_EL2.{IMO,FMO} route physical IRQ/FIQ to EL2 while the guest
     * (EL1) is executing, and exception masks only gate delivery when the target
     * Exception level equals the CURRENTLY EXECUTING Exception level -- since EL2 is
     * not the currently executing EL once we've eret'd into the EL1 guest, EL2's own
     * (permanently masked) DAIF has no effect on that routed delivery. Masking here only
     * defers delivery of any interrupt that becomes pending before the guest exists. */
    unsafe { asm::get_daif_and_disable_irq_fiq() };
    let distributor = init_gic_distributor(&dtb);
    let _gic_cpu_interface = init_gic_cpu_interface(&dtb);
    let gic_hypervisor_interface = init_gic_hypervisor_interface(&dtb);
    let (gicv_base_address, gicv_size) = get_gic_virtual_cpu_interface(&dtb);

    enable_serial_port_interrupt(&PL011_DEVICE.lock(), &distributor);
    /* Called after the PL011_DEVICE lock (held for the whole statement above) has
     * been released: dump_spi_config() prints via println!, which itself needs to
     * lock PL011_DEVICE, so calling it while still holding that lock would
     * self-deadlock (this crate's Mutex is a simple non-reentrant spinlock). */
    if unsafe { PL011_INT_ID } != 0 {
        distributor.dump_spi_config(unsafe { PL011_INT_ID });
    }

    generic_timer::init_generic_timer_global(&dtb);

    /* Prefer a Virtio-Blk device (QEMU's `virt` machine); fall back to a
     * physical SDHCI controller (e.g. Raspberry Pi 4's onboard microSD
     * slot), then to USB mass storage behind the Pi 4's VL805 xHCI
     * controller, when no earlier backend is present. If none are found,
     * report this clearly instead of panicking, so that console/GIC/UART/
     * SMP bring-up can still be verified on such platforms. */
    let Some(mut virtblk) = init_virtio_blk(&dtb)
        .map(BlockBackend::Virtio)
        .or_else(|| init_sdhci(&dtb).map(BlockBackend::Sdhci))
        .or_else(|| init_usb_storage(&dtb).map(BlockBackend::Usb))
    else {
        println!("No supported block-storage device (Virtio-Blk/SDHCI/USB storage) was found.");
        println!("Guest storage/boot is not supported on this platform yet.");
        loop {
            core::hint::spin_loop();
        }
    };
    let fat32 = init_fat32(&mut virtblk);

    let (net, net_int_id, net_mac) = init_physical_net(&dtb);

    let (boot_address, argument) = vm::create_vm(
        &fat32,
        &mut virtblk,
        &distributor,
        &gic_hypervisor_interface,
        gicv_base_address,
        gicv_size,
        net_mac,
    );

    /* The physical VM is now active: it is now safe to enable the
     * Virtio-Net interrupt, since handle_net_rx() requires an active VM. */
    *PHYSICAL_NET.lock() = net;
    if let Some(int_id) = net_int_id {
        enable_net_interrupt(int_id, &distributor);
    }

    /* Check PSCI version.
     *
     * Some platforms (e.g. Raspberry Pi 4's stock firmware) provide no
     * EL3/PSCI firmware at all -- their DTB has no `/psci` node and CPUs
     * are instead brought up via the ARM "spin-table" protocol (see
     * `is_spin_table_enable_method`/`launch_cpu`). Executing `smc` on such
     * hardware is UNDEFINED (there is no EL3 to service it) and traps to
     * the current-EL synchronous vector, which is just an infinite loop
     * (see `exception.rs`), silently hanging the hypervisor with no
     * output at all. Only probe PSCI when the DTB actually advertises it. */
    let has_psci = dtb
        .search_node_by_compatible(b"arm,psci-0.2", None)
        .or_else(|| dtb.search_node_by_compatible(b"arm,psci", None))
        .is_some();
    HAS_PSCI.store(has_psci, Ordering::Relaxed);
    if has_psci {
        let (major_version, minor_version) =
            psci::check_psci_version().expect("PSCI is not supported");
        println!("PSCI version {major_version}.{minor_version}");
    } else {
        println!("PSCI is not present in the devicetree; using spin-table CPU bring-up only.");
    }

    /* Park every other physical CPU described in the DTB so the guest
     * kernel booted below can bring them up itself (via its own PSCI
     * CPU_ON calls, virtualized in `handle_guest_smc`) as additional vCPUs
     * of *this* VM -- true multi-CPU SMP inside a single guest, rather than
     * each physical CPU always hosting its own independent VM. */
    park_secondary_cpus_for_smp(&dtb, has_psci);

    *VIRTIO_BLK.lock() = virtblk;
    unsafe {
        (&raw mut FAT32).as_mut().unwrap().write(fat32);
        (&raw mut DTB).as_mut().unwrap().write(dtb);
    }

    vm::boot_vm(boot_address, argument)
}

/* Parses a u-boot-supplied argv string (always a memory address) as an
 * unsigned integer. Addresses passed to this function are always expressed
 * in hexadecimal, but not always with an explicit "0x" prefix: e.g. u-boot's
 * $kernel_addr_r env var is formatted as "0x00080000", while $fdt_addr on
 * Raspberry Pi hardware (set by the board's own init code, unlike QEMU) is
 * formatted as a bare hex string such as "3af02bb0" with no prefix. Default
 * to hex (not decimal) when no prefix is present, since a decimal address
 * is never a legitimate input here and would otherwise make parsing silently
 * fail on any unprefixed value containing the digits a-f. */
fn str_to_usize(s: &str) -> Option<usize> {
    let radix;
    let start;
    match s.get(0..2) {
        Some("0x") => {
            radix = 16;
            start = s.get(2..);
        }
        Some("0o") => {
            radix = 8;
            start = s.get(2..);
        }
        Some("0b") => {
            radix = 2;
            start = s.get(2..);
        }
        _ => {
            radix = 16;
            start = Some(s);
        }
    }
    usize::from_str_radix(start?, radix).ok()
}

fn init_serial_port(dtb: &dtb::Dtb) -> Result<(), usize> {
    let mut pl011 = None;
    loop {
        pl011 = dtb.search_node_by_compatible(b"arm,pl011", pl011.as_ref());
        match &pl011 {
            Some(d) => {
                if !dtb.is_node_operational(d) {
                    continue;
                } else {
                    break;
                }
            }
            None => {
                return Err(5);
            }
        }
    }
    let pl011 = pl011.unwrap();
    let Some((pl011_base, pl011_range)) = dtb.read_reg_property(&pl011, 0) else {
        return Err(6);
    };
    let pl011_base = dtb.translate_soc_address(pl011_base);

    let interrupts =
        dtb.read_property_as_u32_array(&dtb.get_property(&pl011, b"interrupts").unwrap());
    let mut interrupt_number = 0;
    if u32::from_be(interrupts[0]) == gicv2::DTB_GIC_SPI
        && u32::from_be(interrupts[2]) == gicv2::DTB_GIC_LEVEL
    {
        interrupt_number = gicv2::GIC_SPI_BASE + u32::from_be(interrupts[1]);
    }

    let Ok(pl011) = pl011::Pl011::new(pl011_base, pl011_range) else {
        return Err(7);
    };
    unsafe { PL011_INT_ID = interrupt_number };
    *PL011_DEVICE.lock() = pl011;
    serial::init_default_serial_port(&PL011_DEVICE);
    println!(
        "PL011: base={pl011_base:#X} range={pl011_range:#X} interrupt_id={interrupt_number}"
    );
    Ok(())
}

#[panic_handler]
pub fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("{info}");
    loop {
        core::hint::spin_loop();
    }
}

pub fn setup_memory(dtb: &dtb::Dtb, dtb_address: usize, elf_address: usize, stack_pointer: usize) {
    let mut memory_allocator = MEMORY_ALLOCATOR.lock();
    /* Register every RAM bank described by the DTB: a memory node may carry
     * multiple reg entries and several memory@... nodes may exist. The 8 GiB
     * Pi4, for example, splits RAM into a bank below 0x3B400000, another at
     * 0x40000000..0xFC000000, and the remainder above 4 GiB. Reading only
     * reg[0] of the first node leaves the other banks unknown, and reserving
     * anything in them (e.g. an ELF segment placed there by the bootloader)
     * panics with "Failed to reserve memory for the segment".
     * Banks at/above 4 GiB are skipped: the xHCI/PCIe hardware can only DMA
     * to 32-bit addresses, so that memory is unusable by the drivers here. */
    const DMA_ADDRESS_LIMIT: usize = 1 << 32;
    let mut current = Some(
        dtb.search_node(b"memory", None)
            .expect("Expected memory node."),
    );
    while let Some(memory) = current {
        let mut index = 0;
        while let Some((start, size)) = dtb.read_reg_property(&memory, index) {
            index += 1;
            if size == 0 {
                continue;
            }
            if start >= DMA_ADDRESS_LIMIT {
                println!("Ignore RAM [{start:#X} ~ {:#X}]: above 4 GiB", start + size);
                continue;
            }
            let size = size.min(DMA_ADDRESS_LIMIT - start);
            println!("RAM is [{:#X} ~ {:#X}]", start, start + size);
            memory_allocator
                .free(start, size)
                .expect("Failed to free the RAM");
        }
        current = dtb.search_node(b"memory", Some(&memory));
    }

    /* Exclude the DTB */
    println!(
        "DTB is [{:#X} ~ {:#X}]",
        dtb_address,
        dtb_address + dtb.get_total_size()
    );
    memory_allocator
        .reserve_memory(dtb_address, dtb.get_total_size(), 0)
        .expect("Failed to reserve DTB");

    let elf_header = elf::Elf64Header::new(elf_address).expect("Invalid ELF Header");
    for p in elf_header.get_program_headers() {
        if p.get_segment_type() == elf::ELF_PROGRAM_HEADER_SEGMENT_LOAD {
            println!(
                "Reserve [{:#X} ~ {:#X}]",
                p.get_physical_address(),
                p.get_physical_address() + p.get_memory_size()
            );
            memory_allocator
                .reserve_memory(
                    p.get_physical_address() as usize,
                    p.get_memory_size() as usize,
                    0,
                )
                .expect("Failed to reserve memory for the segment");
        }
    }

    /* Exclude the stack */
    let stack_end = ((stack_pointer - 1) & !(paging::PAGE_SIZE - 1)) + paging::PAGE_SIZE;
    let stack_start = stack_end - STACK_SIZE;
    println!("Reserve [{:#X} ~ {:#X}] for Stack", stack_start, stack_end);
    memory_allocator
        .reserve_memory(stack_start, STACK_SIZE, 0)
        .expect("Failed to reserve memory for the stack");
}

pub fn allocate_pages(
    number_of_pages: usize,
    align: usize,
) -> Result<usize, memory_allocator::MemoryError> {
    match MEMORY_ALLOCATOR
        .lock()
        .allocate(number_of_pages << paging::PAGE_SHIFT, align)
    {
        Ok(a) => Ok(a),
        Err(e) => {
            println!("Failed to allocate memory: {:?}", e);
            Err(e)
        }
    }
}

pub fn free_pages(address: usize, number_of_pages: usize) {
    let _ = MEMORY_ALLOCATOR
        .lock()
        .free(address, number_of_pages << paging::PAGE_SHIFT);
}

/// Compatible strings for the GICv2 (with Virtualization Extensions) node.
/// QEMU's `virt` machine advertises "arm,cortex-a15-gic", while Raspberry Pi 4
/// (BCM2711)'s GIC-400 advertises "arm,gic-400". Both expose the same 4 MMIO
/// regions (Distributor / CPU Interface / Hypervisor Interface / Virtual CPU
/// Interface) in the same order, so the rest of the driver is unchanged.
const GIC_COMPATIBLE_LIST: &[&[u8]] = &[b"arm,cortex-a15-gic", b"arm,gic-400"];

fn find_gic_node(dtb: &dtb::Dtb) -> dtb::DtbNode {
    for compatible in GIC_COMPATIBLE_LIST {
        if let Some(node) = dtb.search_node_by_compatible(compatible, None) {
            return node;
        }
    }
    panic!("No compatible GICv2 node found in the DTB");
}

fn init_gic_distributor(dtb: &dtb::Dtb) -> gicv2::GicDistributor {
    let gic_node = find_gic_node(dtb);
    let (base_address, size) = dtb.read_reg_property(&gic_node, 0).unwrap();
    let base_address = dtb.translate_soc_address(base_address);
    println!("GIC Distributor's Base Address: {:#X}", base_address);
    let gic_distributor = gicv2::GicDistributor::new(base_address, size).unwrap();
    gic_distributor.init();
    gic_distributor
}

fn init_gic_cpu_interface(dtb: &dtb::Dtb) -> gicv2::GicCpuInterface {
    let gic_node = find_gic_node(dtb);
    let (base_address, size) = dtb.read_reg_property(&gic_node, 1).unwrap();
    let base_address = dtb.translate_soc_address(base_address);
    if size < gicv2::GicCpuInterface::GICC_MMIO_SIZE {
        panic!("Invalid GICC Size: {:#X}", size);
    }
    println!("GIC CPU Interface's Base Address: {:#X}", base_address);
    let gic_cpu_interface = gicv2::GicCpuInterface::new(base_address);
    gic_cpu_interface.init();
    gic_cpu_interface
}

fn init_gic_hypervisor_interface(dtb: &dtb::Dtb) -> gicv2::GicHypervisorInterface {
    let gic_node = find_gic_node(dtb);
    let (base_address, size) = dtb.read_reg_property(&gic_node, 2).unwrap();
    let base_address = dtb.translate_soc_address(base_address);
    if size < gicv2::GicHypervisorInterface::GICH_MMIO_SIZE {
        panic!("Invalid GICH Size: {:#X}", size);
    }
    println!("GIC Hypervisor Interface's Base Address: {:#X}", base_address);
    gicv2::GicHypervisorInterface::new(base_address)
}

/// Gets the physical address of the GICv2 Virtual CPU Interface (GICV).
/// (Used to map it via Stage 2 passthrough to the address corresponding to the guest's GICC)
fn get_gic_virtual_cpu_interface(dtb: &dtb::Dtb) -> (usize, usize) {
    let gic_node = find_gic_node(dtb);
    let (base_address, size) = dtb.read_reg_property(&gic_node, 3).unwrap();
    (dtb.translate_soc_address(base_address), size)
}

fn enable_serial_port_interrupt(pl011: &pl011::Pl011, distributor: &gicv2::GicDistributor) {
    let int_id = unsafe { PL011_INT_ID };
    if int_id == 0 {
        println!("PL011 does not support interrupt.");
        return;
    }
    distributor.set_group(int_id, gicv2::GicGroup::NonSecureGroup1);
    distributor.set_priority(int_id, 0x00);
    distributor.set_target(int_id, distributor.get_own_target());
    distributor.set_trigger_mode(int_id, true);
    distributor.set_pending(int_id, false);
    distributor.set_enable(int_id, true);
    pl011.enable_interrupt();
}

fn init_virtio_blk(dtb: &dtb::Dtb) -> Option<virtio_blk::VirtioBlk> {
    let mut virtio = None;
    loop {
        virtio = dtb.search_node_by_compatible(b"virtio,mmio", virtio.as_ref());
        match &virtio {
            Some(virtio) => {
                if dtb.is_node_operational(virtio) {
                    let (base_address, _) = dtb.read_reg_property(virtio, 0).unwrap();
                    if let Ok(blk) = virtio_blk::VirtioBlk::new(base_address) {
                        return Some(blk);
                    }
                }
            }
            None => {
                return None;
            }
        }
    }
}

/// Searches the DTB for Raspberry Pi 4's BCM2711 GENET controller and
/// initializes the physical RX/TX data path.
fn init_genet(dtb: &dtb::Dtb) -> Option<genet::Genet> {
    const GENET_COMPATIBLE_LIST: &[&[u8]] = &[b"brcm,bcm2711-genet-v5", b"brcm,genet-v5"];

    for compatible in GENET_COMPATIBLE_LIST {
        let mut node = None;
        loop {
            node = dtb.search_node_by_compatible(compatible, node.as_ref());
            match &node {
                Some(genet_node) => {
                    if !dtb.is_node_operational(genet_node) {
                        continue;
                    }
                    let Some((base_address, range)) = dtb.read_reg_property(genet_node, 0) else {
                        continue;
                    };
                    let base_address = dtb.translate_soc_address(base_address);
                    let Ok(net) = genet::Genet::new(base_address, range) else {
                        println!("GENET: probe failed at base={base_address:#X}");
                        continue;
                    };

                    let (phy_id1, phy_id2) = net.get_phy_id();
                    println!(
                        "GENET: base={:#X} phy_addr={} phy_id={:04X}:{:04X}",
                        net.get_base_address(),
                        net.get_phy_address(),
                        phy_id1,
                        phy_id2
                    );

                    match net.wait_link_up(3_000) {
                        Ok(link) if link.is_up() => {
                            println!(
                                "GENET link is up (phy_link={} mac_link={})",
                                link.phy_link, link.mac_link
                            );
                        }
                        Ok(link) => {
                            println!(
                                "GENET link is down (phy_link={} mac_link={})",
                                link.phy_link, link.mac_link
                            );
                        }
                        Err(()) => {
                            println!("GENET: failed to read link status");
                        }
                    }
                    return Some(net);
                }
                None => break,
            }
        }
    }
    println!("GENET device is not present.");
    None
}

/// Searches the DTB for an SDHCI-compatible microSD controller (e.g.
/// Raspberry Pi 4's EMMC2, `compatible = "brcm,bcm2711-emmc2"`) and, if
/// found and operational, initializes it and probes for a card.
fn init_sdhci(dtb: &dtb::Dtb) -> Option<drivers::sdhci::Sdhci> {
    const SDHCI_COMPATIBLE_LIST: &[&[u8]] = &[
        b"brcm,bcm2711-emmc2",
        b"brcm,sdhci-brcmstb",
        b"generic-sdhci",
    ];
    for compatible in SDHCI_COMPATIBLE_LIST {
        let mut node = None;
        loop {
            node = dtb.search_node_by_compatible(compatible, node.as_ref());
            match &node {
                Some(sdhci_node) => {
                    if dtb.is_node_operational(sdhci_node)
                        && let Some((base_address, _)) = dtb.read_reg_property(sdhci_node, 0)
                    {
                        let base_address = dtb.translate_soc_address(base_address);
                        match drivers::sdhci::Sdhci::new(base_address) {
                            Ok(sdhci) => return Some(sdhci),
                            Err(()) => println!("Failed to initialize the SDHCI controller."),
                        }
                    }
                }
                None => break,
            }
        }
    }
    None
}

/// Searches the DTB for Raspberry Pi 4's PCIe root complex
/// (`compatible = "brcm,bcm2711-pcie"`) and, if present and operational,
/// initializes the already-enumerated downstream VL805 xHCI controller and
/// probes a directly attached USB mass-storage device. Failures are reported
/// and returned as `None` so SDHCI/Virtio fallback remains available.
fn init_usb_storage(dtb: &dtb::Dtb) -> Option<usb_mass_storage::UsbMassStorage> {
    let Some(node) = dtb.search_node_by_compatible(b"brcm,bcm2711-pcie", None) else {
        println!("PCIe root complex is not present.");
        return None;
    };
    if !dtb.is_node_operational(&node) {
        println!("PCIe root complex is disabled in the DTB.");
        return None;
    }
    let Ok(pcie) = pcie_brcm::PcieBrcm::new(dtb, &node) else {
        println!("Failed to initialize the PCIe root complex.");
        return None;
    };
    if !pcie.is_link_up() {
        println!("PCIe link is down.");
        return None;
    }
    let Some(xhci_pci) = pcie.find_xhci_device() else {
        println!("No downstream xHCI PCI device was found.");
        return None;
    };
    println!(
        "PCIe xHCI device {:04X}:{:04X} at CPU MMIO {:#X}",
        xhci_pci.vendor_id, xhci_pci.device_id, xhci_pci.cpu_mmio_base
    );
    let Ok(xhci) = xhci::Xhci::new(xhci_pci.cpu_mmio_base, xhci_pci.dma_offset) else {
        println!("Failed to initialize the xHCI controller.");
        return None;
    };
    let Ok(device) = xhci.probe_mass_storage() else {
        println!("No USB mass-storage device was found on xHCI.");
        return None;
    };
    match usb_mass_storage::UsbMassStorage::new(device) {
        Ok(storage) => Some(storage),
        Err(()) => {
            println!("Failed to initialize the USB mass-storage device.");
            None
        }
    }
}

/// Default MAC address used when the physical Virtio-Net device does not
/// support VIRTIO_NET_F_MAC (locally administered, QEMU/virtual convention).
const DEFAULT_MAC_ADDRESS: [u8; 6] = [0x52, 0x54, 0x00, 0x12, 0x34, 0x56];

/// Initializes the physical network backend.
///
/// 1. QEMU: use real virtio-net hardware as before.
/// 2. Raspberry Pi 4: fall back to BCM2711 GENET.
fn init_physical_net(dtb: &dtb::Dtb) -> (Option<PhysicalNet>, Option<u32>, [u8; 6]) {
    let mut node = None;
    loop {
        node = dtb.search_node_by_compatible(b"virtio,mmio", node.as_ref());
        match &node {
            Some(virtio) => {
                if dtb.is_node_operational(virtio)
                    && let Some((base_address, _)) = dtb.read_reg_property(virtio, 0)
                    && let Ok(net) = virtio_net::VirtioNet::new(base_address)
                {
                    let mut int_id = None;
                    if let Some(interrupts_property) = dtb.get_property(virtio, b"interrupts") {
                        let interrupts = dtb.read_property_as_u32_array(&interrupts_property);
                        if u32::from_be(interrupts[0]) == gicv2::DTB_GIC_SPI {
                            int_id = Some(gicv2::GIC_SPI_BASE + u32::from_be(interrupts[1]));
                        }
                    }
                    let backend = PhysicalNet::Virtio(net);
                    let mac = backend.get_mac_address();
                    let mac = if mac == [0u8; 6] {
                        DEFAULT_MAC_ADDRESS
                    } else {
                        mac
                    };
                    return (Some(backend), int_id, mac);
                }
            }
            None => {
                break;
            }
        }
    }

    let Some(net) = init_genet(dtb) else {
        println!("Virtio-Net/GENET physical backend is not present.");
        return (None, None, DEFAULT_MAC_ADDRESS);
    };
    let backend = PhysicalNet::Genet(net);
    let mac = backend.get_mac_address();
    (Some(backend), None, if mac == [0u8; 6] { DEFAULT_MAC_ADDRESS } else { mac })
}

/// Enables the physical Virtio-Net RX interrupt. Must only be called once a
/// VM is active, since the handler forwards packets into the current VM.
fn enable_net_interrupt(int_id: u32, distributor: &gicv2::GicDistributor) {
    distributor.set_group(int_id, gicv2::GicGroup::NonSecureGroup1);
    distributor.set_priority(int_id, 0x00);
    distributor.set_target(int_id, distributor.get_own_target());
    /* Although the DTB advertises this line as edge-triggered, QEMU's
     * virtio-mmio model keeps the physical IRQ asserted for as long as
     * VIRTIO_MMIO_INTERRUPT_STATUS is non-zero (level semantics); configure
     * the physical GIC accordingly so it stops re-presenting the interrupt
     * once acknowledged (see VirtioNet::poll_rx's INTERRUPT_ACK write). */
    distributor.set_trigger_mode(int_id, true);
    distributor.set_pending(int_id, false);
    distributor.set_enable(int_id, true);
    unsafe { VIRTIO_NET_INT_ID = int_id };
}

/// Called from the physical Virtio-Net IRQ handler: drains every received
/// Ethernet frame and forwards it to the currently active VM.
///
/// Must never hold `PHYSICAL_NET`'s lock while calling into
/// `vm::input_net_packet` (which locks the guest's own `virtio_net_mmio`):
/// `VirtioNetMmio::process_tx` (a guest-triggered MMIO trap) locks
/// `virtio_net_mmio` first and then `PHYSICAL_NET` to send the packet out.
/// Now that multiple vCPUs run truly concurrently on separate pCPUs, this
/// function's physical-RX IRQ handler and a guest's TX MMIO trap can run
/// on different pCPUs at the same time; acquiring the same two locks in
/// opposite order would be a classic AB-BA deadlock (observed as some
/// vCPUs' physical interrupts -- including their own local timer, since
/// the deadlocked pCPU is stuck spinning inside this exception handler
/// with IRQs masked -- silently and permanently stopping). Polling the
/// physical device and forwarding the received packet are therefore kept
/// as separate steps, each taking only one of the two locks at a time.
fn handle_net_rx() {
    const MAX_PACKETS_PER_CALL: usize = 16;
    let mut buffer = [0u8; drivers::virtio_net::VIRTIO_NET_RX_BUFFER_SIZE];
    let mut processed = 0usize;
    while processed < MAX_PACKETS_PER_CALL {
        let length = {
            let mut net = PHYSICAL_NET.lock();
            let Some(net) = net.as_mut() else {
                return;
            };
            let Some(length) = net.poll_rx(&mut buffer) else {
                break;
            };
            length
        };
        vm::input_net_packet(&buffer[..length]);
        processed += 1;
    }
}

fn needs_net_polling_on_wfx() -> bool {
    PHYSICAL_NET
        .lock()
        .as_ref()
        .map(PhysicalNet::requires_wfx_polling)
        .unwrap_or(false)
}

fn get_guest_net_mac() -> [u8; 6] {
    PHYSICAL_NET
        .lock()
        .as_ref()
        .map(PhysicalNet::get_mac_address)
        .unwrap_or(DEFAULT_MAC_ADDRESS)
}

pub fn init_fat32(blk: &mut dyn drivers::block_device::BlockDevice) -> fat32::Fat32 {
    #[repr(C, packed)]
    #[derive(Clone, Copy)]
    struct PartitionTableEntry {
        boot_flag: u8,
        first_sector: [u8; 3],
        partition_type: u8,
        last_sector: [u8; 3],
        first_sector_lba: u32,
        number_of_sectors: u32,
    }
    const PARTITION_TABLE_BASE: usize = 0x1BE;
    /* Read the MBR */
    #[repr(align(4))]
    struct AlignedBuffer([u8; 512]);
    let mut mbr = AlignedBuffer([0; 512]);
    blk.read(&mut mbr as *mut _ as usize, 0, 512)
        .expect("Failed to read first 512bytes");
    let mbr = &mbr.0;
    /* Verify the BOOT signature */
    assert_eq!(u16::from_le_bytes([mbr[510], mbr[511]]), 0xAA55);

    /* Parse the partition table. PARTITION_TABLE_BASE (0x1BE) is not
     * 4-byte aligned, so the entries (which contain u32 fields) cannot be
     * referenced in place; read them out as `packed` (alignment-1) values
     * via a by-value copy instead, which is always sound regardless of
     * source alignment. */
    let partition_table = unsafe {
        core::ptr::read_unaligned(
            &mbr[PARTITION_TABLE_BASE] as *const _ as *const [PartitionTableEntry; 4],
        )
    };
    let mut fat32 = Err(());
    for e in partition_table {
        if e.partition_type == 0x0C {
            fat32 = fat32::Fat32::new(blk, e.first_sector_lba as usize, 512);
            break;
        }
    }

    let fat32 = fat32.expect("The FAT32 Partition is not found!");
    fat32.list_files();

    fat32
}

unsafe impl GlobalAlloc for GlobalAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        match MEMORY_ALLOCATOR
            .lock()
            .allocate(layout.size(), layout.align().trailing_zeros() as usize)
        {
            Ok(address) => address as *mut u8,
            Err(e) => {
                println!("Failed to allocate memory: {:?}", e);
                core::ptr::null_mut()
            }
        }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        let _ = MEMORY_ALLOCATOR.lock().free(ptr as usize, layout.size());
    }
}

/// Brings up every other physical CPU described in the DTB (i.e. every
/// `cpu` node whose affinity is not this, the boot CPU's) and parks it (see
/// [`smp_park_main`]), rather than leaving it idle until a debug console
/// command manually starts a brand-new, independent VM on it (`launch_cpu`).
/// Each parked CPU can later be claimed by the guest kernel's own
/// (virtualized) PSCI CPU_ON call -- see [`handle_guest_cpu_on`] -- letting
/// a single guest use multiple CPUs.
///
/// `has_psci` mirrors the same DTB-presence check `entry_main` already
/// performs before ever executing `smc`: on platforms with no EL3/PSCI
/// firmware at all (only `enable-method = "spin-table"` cores), issuing
/// `smc` is undefined behaviour, so non-spin-table nodes are simply skipped
/// (and reported) instead when `has_psci` is false.
fn park_secondary_cpus_for_smp(dtb: &dtb::Dtb, has_psci: bool) {
    let current_affinity = asm::mpidr_to_affinity(asm::get_mpidr_el1());
    let mut cpu_node = None;
    while let Some(cpu) = dtb.search_node(b"cpu", cpu_node.as_ref()) {
        if let Some((affinity, _)) = dtb.read_reg_property(&cpu, 0)
            && current_affinity != affinity as u64
        {
            let affinity = affinity as u64;
            let stack_address = allocate_pages(STACK_SIZE >> paging::PAGE_SHIFT, 0)
                .expect("Failed to allocate memory")
                + STACK_SIZE;
            if is_spin_table_enable_method(dtb, &cpu) {
                let Some(release_address) = read_cpu_release_address(dtb, &cpu) else {
                    println!(
                        "CPU(Affinity: {:#X}) is missing a valid cpu-release-addr; \
                         not available for guest SMP",
                        affinity
                    );
                    free_pages(stack_address - STACK_SIZE, STACK_SIZE >> paging::PAGE_SHIFT);
                    cpu_node = Some(cpu);
                    continue;
                };
                println!(
                    "Parking CPU(Affinity: {:#X}) for guest SMP via spin-table \
                     (release address: {:#X})",
                    affinity, release_address
                );
                psci::spin_table_cpu_on(
                    release_address,
                    stack_address as u64,
                    asm::smp_park_entry as *const () as u64,
                );
                STACK_TOPS.lock().push((affinity, stack_address as u64));
            } else if has_psci {
                match psci::cpu_on(
                    affinity,
                    asm::smp_park_entry as *const () as usize as u64,
                    stack_address as u64,
                ) {
                    Ok(_) => {
                        println!("Parking CPU(Affinity: {:#X}) for guest SMP via PSCI", affinity);
                        STACK_TOPS.lock().push((affinity, stack_address as u64));
                    }
                    Err(e) => {
                        println!(
                            "Failed to park CPU(Affinity: {:#X}) for guest SMP: {:?}",
                            affinity, e
                        );
                        free_pages(stack_address - STACK_SIZE, STACK_SIZE >> paging::PAGE_SHIFT);
                    }
                }
            } else {
                println!(
                    "CPU(Affinity: {:#X}) requires PSCI, which this platform does not \
                     advertise; not available for guest SMP",
                    affinity
                );
                free_pages(stack_address - STACK_SIZE, STACK_SIZE >> paging::PAGE_SHIFT);
            }
        }
        cpu_node = Some(cpu);
    }
}

/// Entry point reached (via [`asm::smp_park_entry`]) by a physical CPU
/// brought up by [`park_secondary_cpus_for_smp`]. Registers itself as
/// available, then parks (`wfe`) until a guest-issued PSCI CPU_ON (see
/// [`handle_guest_cpu_on`]) hands it a vCPU to run, at which point it joins
/// that VM (see `vm::activate_vm_on_this_pcpu`) and `eret`s into the
/// guest-supplied entry point -- exactly like a real ARM SMP secondary
/// core coming online.
extern "C" fn smp_park_main() -> ! {
    let current_el = asm::get_currentel() >> 2;
    assert_eq!(current_el, 2);

    let affinity = asm::mpidr_to_affinity(asm::get_mpidr_el1());
    let parked = Arc::new(ParkedCpu {
        affinity,
        launch: Mutex::new(None),
    });
    PARKED_CPUS.lock().push_back(parked.clone());

    let request = loop {
        if let Some(request) = parked.launch.lock().take() {
            break request;
        }
        asm::wfe();
    };

    /* Per-physical-CPU hardware state (EL2 system registers, the physical
     * GIC CPU/Hypervisor Interfaces, the vGIC's per-core PPI setup) is
     * banked per pCPU, so it must be (re-)configured here even though this
     * VM's Stage 2 table/RAM/MMIO devices/GICD are already fully set up by
     * whichever pCPU originally created it. Re-running the (idempotent)
     * GICD/GICC/GICH init sequence used by every other pCPU is harmless:
     * it only ever (re-)writes the same enable bits, never disables
     * anything another pCPU already depends on. */
    exception::setup_exception();
    let dtb = unsafe { (&raw const DTB).as_ref().unwrap().assume_init_ref() };
    let distributor = init_gic_distributor(dtb);
    let _gic_cpu_interface = init_gic_cpu_interface(dtb);
    let gic_hypervisor_interface = init_gic_hypervisor_interface(dtb);
    vgic::init_vgic(&gic_hypervisor_interface, &distributor);
    /* The Generic Timer's enable bit, like its interrupt, is banked per pCPU, so it must be
     * (re-)armed here too -- otherwise this pCPU's own local timer PPI never actually fires,
     * leaving this vCPU without a scheduler tick once it goes idle. */
    generic_timer::init_generic_timer_local(&distributor);

    vm::activate_vm_on_this_pcpu(&request.vm);
    println!(
        "CPU(Affinity: {:#X}) joined VM{} as an additional vCPU (entry point: {:#X})",
        affinity,
        request.vm.vm_id(),
        request.entry_point
    );
    vm::boot_vm(request.entry_point, request.context_id)
}

/// Virtualizes the guest's own PSCI SMC calls (trapped via HCR_EL2.TSC,
/// since this DTB's `psci` node advertises `method = "smc"`). CPU_ON and
/// CPU_OFF are both virtualized, so the guest kernel can bring up and
/// retire additional vCPUs of *this same* VM on the physical CPUs parked
/// by [`park_secondary_cpus_for_smp`] -- true multi-vCPU SMP, as opposed to
/// the `boot`/`spawn` debug console commands' separate-VM-per-pCPU model.
/// Every other PSCI function (SYSTEM_OFF, SYSTEM_RESET, version queries,
/// ...) is passed through to the real firmware unmodified when the machine
/// actually has EL3 PSCI firmware ([`HAS_PSCI`]), e.g. so a guest-issued
/// shutdown/reboot still powers off/resets real hardware. On machines
/// without EL3 firmware (e.g. the Raspberry Pi 4, where `smc` executed at
/// EL2 is UNDEFINED and would fault the hypervisor itself), a minimal
/// PSCI v0.2 implementation is emulated instead: PSCI_VERSION reports v0.2
/// so the guest never attempts v1.0-only calls such as PSCI_FEATURES,
/// SYSTEM_OFF/SYSTEM_RESET park the calling vCPU (there is no firmware to
/// perform a real power-off/reset), and everything else returns
/// NOT_SUPPORTED.
pub fn handle_guest_smc(registers: &mut exception::Registers) {
    if registers.x0 == psci::PSCI_CPU_ON {
        registers.x0 =
            handle_guest_cpu_on(registers.x1, registers.x2 as usize, registers.x3 as usize);
        unsafe { asm::advance_elr_el2() };
    } else if registers.x0 == psci::PSCI_CPU_OFF {
        /* Never actually returns here: `handle_guest_cpu_off` either directly
         * substitutes `registers`/ELR_EL2/SPSR_EL2 with another queued VCPU's
         * context (so the shared exception epilogue erets into *that* VCPU
         * instead of advancing past this one, which no longer exists), or
         * diverges entirely to re-park this physical CPU. Either way, the
         * calling VCPU's own CPU_OFF never "returns" a value to it, per the
         * PSCI spec (CPU_OFF only returns to the caller on failure, which
         * this hypervisor's virtualized version cannot produce). */
        handle_guest_cpu_off(registers);
    } else if HAS_PSCI.load(Ordering::Relaxed) {
        registers.x0 = unsafe { asm::smc(registers.x0, registers.x1, registers.x2, registers.x3) };
        unsafe { asm::advance_elr_el2() };
    } else {
        /* No EL3/PSCI firmware exists on this machine, so forwarding via
         * `smc` is impossible (it is UNDEFINED at EL2 there). Emulate a
         * minimal PSCI v0.2 instead -- see this function's doc comment. */
        registers.x0 = match registers.x0 {
            psci::PSCI_VERSION => 2, /* Report PSCI v0.2 */
            psci::PSCI_SYSTEM_OFF | psci::PSCI_SYSTEM_RESET => {
                println!(
                    "Guest requested PSCI SYSTEM_OFF/SYSTEM_RESET, but this machine has no \
                     EL3 firmware to perform it; parking the vCPU instead."
                );
                loop {
                    asm::wfe();
                }
            }
            _ => (-1i32) as u64, /* NOT_SUPPORTED */
        };
        unsafe { asm::advance_elr_el2() };
    }
}

/// Services a guest-issued PSCI CPU_ON (see [`handle_guest_smc`]): finds
/// the physical CPU matching `target_cpu`'s affinity among those parked by
/// [`park_secondary_cpus_for_smp`], removes it from the parked pool, hands
/// it the requested entry point/context id (joining the *caller's* VM --
/// i.e. whichever vCPU issued this CPU_ON, via `vm::get_current_vm`), and
/// wakes it with `sev`. Returns the PSCI return code to place in x0 (0 =
/// SUCCESS, or a negative PSCI error code).
fn handle_guest_cpu_on(target_cpu: u64, entry_point: usize, context_id: usize) -> u64 {
    const PSCI_ALREADY_ON: u64 = (-4i32) as u32 as u64;

    let target_affinity = asm::mpidr_to_affinity(target_cpu);
    let mut parked_cpus = PARKED_CPUS.lock();
    let Some(index) = parked_cpus
        .iter()
        .position(|cpu| cpu.affinity == target_affinity)
    else {
        /* Either an unknown affinity, or this CPU was already claimed by
         * an earlier CPU_ON (this hypervisor never reports a vCPU back to
         * the parked pool once claimed, so "not parked" always means
         * "already on" here). */
        return PSCI_ALREADY_ON;
    };
    let mut list_tail = parked_cpus.split_off(index);
    let cpu = list_tail.pop_front().unwrap();
    parked_cpus.append(&mut list_tail);
    drop(parked_cpus);

    *cpu.launch.lock() = Some(SmpLaunchRequest {
        vm: vm::get_current_vm(),
        entry_point,
        context_id,
    });
    unsafe { asm::sev() };
    0 /* PSCI_SUCCESS */
}

/// Services a guest-issued PSCI CPU_OFF (see [`handle_guest_smc`]),
/// virtualizing it the same way [`handle_guest_cpu_on`] virtualizes CPU_ON,
/// instead of forwarding it to the real firmware and permanently powering
/// off this physical CPU: either -- if another, independent VM is already
/// queued on this same pCPU (under the debug `boot`/`spawn` console
/// commands' separate-VM-per-pCPU model, see
/// `vm::find_other_same_affinity_vcpu`) -- switches straight to it
/// (`vm::switch_to_other_vcpu_on_retire`), letting the exception epilogue
/// `eret` into that VM instead of the VCPU that issued this CPU_OFF; or, if
/// none is queued, re-parks the physical CPU (`asm::reset_stack_and_park`)
/// so a future guest-issued CPU_ON can reclaim it, exactly like it was
/// before ever being claimed by [`handle_guest_cpu_on`].
///
/// Deliberately does *not* remove this VCPU's `VM_LIST` entry (unlike an
/// earlier, buggy version of this function): for a true-SMP guest, every
/// physical CPU that joined this same VM via
/// `vm::activate_vm_on_this_pcpu` (i.e. every additional vCPU brought up
/// via [`handle_guest_cpu_on`]) shares that *one* `VM_LIST` entry, so
/// deleting it here -- just because *this* pCPU happens to be retiring --
/// would leave every other, still-actively-running pCPU's next
/// `vm::get_current_vm()` (keyed by that same now-missing `vm_id`)
/// panicking. See `vm::switch_to_other_vcpu_on_retire`'s doc comment for
/// the full reasoning.
///
/// Never returns to the caller: per the PSCI spec, CPU_OFF only returns a
/// value on failure, and this virtualized implementation cannot fail.
fn handle_guest_cpu_off(registers: &mut exception::Registers) {
    let retiring_vm_id = asm::get_tpidr_el2() as usize;

    if vm::switch_to_other_vcpu_on_retire(retiring_vm_id, registers) {
        /* `registers`/ELR_EL2/SPSR_EL2 now hold the other VM's already
         * correctly-positioned resume state, so just let this function
         * return normally: the caller (`handle_guest_smc`) must NOT advance
         * ELR_EL2 again afterwards, since that would incorrectly skip an
         * instruction of the VM we just switched to rather than the one
         * that actually issued this CPU_OFF. */
        return;
    }

    /* No other VM is queued on this pCPU: re-park it exactly as
     * `park_secondary_cpus_for_smp` originally did, awaiting a future
     * guest-issued CPU_ON. Resetting `sp` back to this pCPU's original
     * stack top (instead of just calling `smp_park_main()` from here)
     * avoids leaking this trap's exception frame and call stack on every
     * such CPU_OFF/CPU_ON cycle. */
    let affinity = asm::mpidr_to_affinity(asm::get_mpidr_el1());
    let stack_top = STACK_TOPS
        .lock()
        .iter()
        .find(|(cpu_affinity, _)| *cpu_affinity == affinity)
        .map(|(_, stack_top)| *stack_top)
        .expect("Re-parking CPU with no recorded stack top");
    asm::reset_stack_and_park(stack_top);
}

pub fn launch_cpu() -> bool {
    let dtb = unsafe { (&raw const DTB).as_ref().unwrap().assume_init_ref() };
    let mut cpu_node = None;
    let current_affinity = asm::mpidr_to_affinity(asm::get_mpidr_el1());
    let stack_address = allocate_pages(STACK_SIZE >> paging::PAGE_SHIFT, 0)
        .expect("Failed to allocate memory")
        + STACK_SIZE;
    while let Some(cpu) = dtb.search_node(b"cpu", cpu_node.as_ref()) {
        if let Some((affinity, _)) = dtb.read_reg_property(&cpu, 0)
            && current_affinity != affinity as u64
        {
            if is_spin_table_enable_method(dtb, &cpu) {
                /* Platforms without PSCI firmware (e.g. Raspberry Pi 4's
                 * stock firmware) bring secondary cores up through the
                 * ARM "spin-table" protocol instead. There is no
                 * acknowledgement from the platform, so assume success
                 * once the release address has been armed. */
                let Some(release_address) = read_cpu_release_address(dtb, &cpu) else {
                    println!(
                        "CPU(Affinity: {:#X}) is missing a valid cpu-release-addr",
                        affinity
                    );
                    cpu_node = Some(cpu);
                    continue;
                };
                println!(
                    "Starting CPU(Affinity: {:#X}) via spin-table (release address: {:#X})",
                    affinity, release_address
                );
                psci::spin_table_cpu_on(
                    release_address,
                    stack_address as u64,
                    asm::core_entry as *const () as u64,
                );
                return true;
            }
            match psci::cpu_on(
                affinity as u64,
                asm::core_entry as *const fn() as usize as u64,
                stack_address as u64,
            ) {
                Ok(_) => return true,
                Err(PsciErrorCodes::AlreadyOn) => { /* Search for the next node */ }
                Err(e) => {
                    println!("Failed to start CPU(Affinity: {:#X}): {:?}", affinity, e);
                }
            }
        }
        cpu_node = Some(cpu);
    }
    free_pages(stack_address - STACK_SIZE, STACK_SIZE >> paging::PAGE_SHIFT);
    false
}

/// Creates an additional VCPU/VM on the pCPU currently executing this
/// function, as opposed to [`launch_cpu`] which starts a brand-new
/// physical core. Unlike [`launch_cpu`] + [`vm::boot_vm`], this never
/// `eret`s: the new VCPU is only *queued* behind whichever VCPU is
/// currently active on this pCPU (see `vm::create_vm`'s
/// `is_first_on_this_pcpu` check) and starts running for the first time
/// only once that VCPU cooperatively yields via a guest WFI/WFE trap (see
/// `vm::try_yield_to_next_vcpu`).
pub fn spawn_vcpu_on_current_pcpu() {
    let dtb = unsafe { (&raw const DTB).as_ref().unwrap().assume_init_ref() };
    let distributor = init_gic_distributor(dtb);
    let gic_hypervisor_interface = init_gic_hypervisor_interface(dtb);
    let (gicv_base_address, gicv_size) = get_gic_virtual_cpu_interface(dtb);
    let fat32 = unsafe { (&raw const FAT32).as_ref().unwrap().assume_init_ref() };

    let _ = vm::create_vm(
        fat32,
        &mut *VIRTIO_BLK.lock(),
        &distributor,
        &gic_hypervisor_interface,
        gicv_base_address,
        gicv_size,
        get_guest_net_mac(),
    );
}

/// Checks whether a `cpu` DTB node advertises the ARM "spin-table" boot
/// protocol (`enable-method = "spin-table"`) instead of PSCI.
fn is_spin_table_enable_method(dtb: &dtb::Dtb, cpu: &dtb::DtbNode) -> bool {
    let Some(property) = dtb.get_property(cpu, b"enable-method") else {
        return false;
    };
    dtb.read_property_as_u8_array(&property).starts_with(b"spin-table")
}

/// Reads a `cpu` DTB node's `cpu-release-addr` property (always encoded as
/// a single 64-bit big-endian value, regardless of the parent bus's
/// `#address-cells`, per the ARM spin-table boot protocol binding).
fn read_cpu_release_address(dtb: &dtb::Dtb, cpu: &dtb::DtbNode) -> Option<usize> {
    let property = dtb.get_property(cpu, b"cpu-release-addr")?;
    let cells = dtb.read_property_as_u32_array(&property);
    if cells.len() < 2 {
        return None;
    }
    let high = u32::from_be(cells[0]) as u64;
    let low = u32::from_be(cells[1]) as u64;
    Some(((high << 32) | low) as usize)
}

extern "C" fn core_main() -> ! {
    let current_el = asm::get_currentel() >> 2;
    assert_eq!(current_el, 2);

    exception::setup_exception();
    let dtb = unsafe { (&raw const DTB).as_ref().unwrap().assume_init_ref() };
    let distributor = init_gic_distributor(dtb);
    let _gic_cpu_interface = init_gic_cpu_interface(dtb);
    let gic_hypervisor_interface = init_gic_hypervisor_interface(dtb);
    let (gicv_base_address, gicv_size) = get_gic_virtual_cpu_interface(dtb);

    let (boot_address, argument) = vm::create_vm(
        unsafe { (&raw const FAT32).as_ref().unwrap().assume_init_ref() },
        &mut *VIRTIO_BLK.lock(),
        &distributor,
        &gic_hypervisor_interface,
        gicv_base_address,
        gicv_size,
        get_guest_net_mac(),
    );
    vm::boot_vm(boot_address, argument)
}

/// Services the physical PL011 after its interrupt was signalled. Acknowledges the
/// UART's RX/RX-timeout interrupt *before* draining the FIFO: the reverse order
/// would silently drop a byte that arrives between the last `getc` and the
/// acknowledge, since the clear would then wipe the interrupt raised for a byte
/// still sitting unread in the FIFO. Clearing first can at worst leave the line
/// re-asserted for a byte this drain already consumed, which merely costs one
/// extra interrupt that finds the FIFO empty.
fn handle_uart_interrupt() {
    PL011_DEVICE.lock().clear_rx_interrupt();
    handle_input(&PL011_DEVICE);
}

fn handle_input(device: &Mutex<dyn SerialDevice>) {
    loop {
        let c = device.lock().getc();
        if c.is_err() {
            println!("Failed to get a character");
            return;
        }
        let c = c.unwrap().unwrap_or(0);
        if c == 0 {
            return;
        }
        if c == CONSOLE_SWITCH_KEY {
            let old = IS_CONSOLE_ACTIVE.fetch_xor(true, Ordering::Relaxed);
            if old {
                /* Deactivate console: overwrite the prompt */
                print!("\r");
            } else {
                /* Activate console: print the prompt */
                CONSOLE.lock().reset_buffer();
            }
        } else if IS_CONSOLE_ACTIVE.load(Ordering::Relaxed) {
            CONSOLE.lock().write(c);
        } else {
            vm::input_uart(c);
        }
    }
}
