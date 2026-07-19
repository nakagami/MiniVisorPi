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
    pub mod gicv2;
    pub mod pl011;
    pub mod sdhci;
    pub mod virtio;
    pub mod virtio_blk;
    pub mod virtio_net;
}
mod elf;
mod exception;
mod fat32;
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
mod vm;

use block_backend::BlockBackend;
use drivers::{generic_timer, gicv2, pl011, virtio_blk, virtio_net};
use lock::Mutex;
use psci::PsciErrorCodes;
use serial::SerialDevice;

use core::alloc::{GlobalAlloc, Layout};
use core::ffi::CStr;
use core::mem::MaybeUninit;
use core::slice;
use core::sync::atomic::{AtomicBool, Ordering};

struct GlobalAllocator {}

/// Global variable storage
static PL011_DEVICE: Mutex<pl011::Pl011> = Mutex::new(pl011::Pl011::invalid());
static mut PL011_INT_ID: u32 = 0;
static MEMORY_ALLOCATOR: Mutex<memory_allocator::MemoryAllocator> =
    Mutex::new(memory_allocator::MemoryAllocator::new());
static VIRTIO_BLK: Mutex<BlockBackend> = Mutex::new(BlockBackend::invalid());
static VIRTIO_NET: Mutex<Option<virtio_net::VirtioNet>> = Mutex::new(None);
static mut VIRTIO_NET_INT_ID: u32 = 0;
static mut FAT32: MaybeUninit<fat32::Fat32> = MaybeUninit::uninit();
#[global_allocator]
static GLOBAL_ALLOCATOR: GlobalAllocator = GlobalAllocator {};
static CONSOLE: Mutex<console::Console> = Mutex::new(console::Console::new());
static IS_CONSOLE_ACTIVE: AtomicBool = AtomicBool::new(false);
static mut DTB: MaybeUninit<dtb::Dtb> = MaybeUninit::uninit();

/// Constants
const STACK_SIZE: usize = 0x10000;
const CONSOLE_SWITCH_KEY: u8 = 0x13; /* Ctrl + S */

#[unsafe(no_mangle)]
extern "C" fn main(argc: usize, argv: *const *const u8) -> usize {
    let stack_pointer = asm::get_stack_pointer() as usize;
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
    setup_memory(&dtb, dtb_address, elf_address, stack_pointer);

    exception::setup_exception();
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
     * slot) when no virtio hardware is present. If neither is found,
     * report this clearly instead of panicking, so that console/GIC/UART/
     * SMP bring-up can still be verified on such platforms. */
    let Some(mut virtblk) = init_virtio_blk(&dtb)
        .map(BlockBackend::Virtio)
        .or_else(|| init_sdhci(&dtb).map(BlockBackend::Sdhci))
    else {
        println!("No supported block-storage device (Virtio-Blk/SDHCI) was found.");
        println!("Guest storage/boot is not supported on this platform yet.");
        loop {
            core::hint::spin_loop();
        }
    };
    let fat32 = init_fat32(&mut virtblk);

    let (net, net_int_id, net_mac) = init_virtio_net(&dtb);

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
    *VIRTIO_NET.lock() = net;
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
    if dtb
        .search_node_by_compatible(b"arm,psci-0.2", None)
        .or_else(|| dtb.search_node_by_compatible(b"arm,psci", None))
        .is_some()
    {
        let (major_version, minor_version) =
            psci::check_psci_version().expect("PSCI is not supported");
        println!("PSCI version {major_version}.{minor_version}");
    } else {
        println!("PSCI is not present in the devicetree; using spin-table CPU bring-up only.");
    }

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
    let memory = dtb
        .search_node(b"memory", None)
        .expect("Expected memory node.");
    let (start, size) = dtb
        .read_reg_property(&memory, 0)
        .expect("Expected reg entry");
    println!("RAM is [{:#X} ~ {:#X}]", start, start + size);
    let mut memory_allocator = MEMORY_ALLOCATOR.lock();
    memory_allocator
        .free(start, size)
        .expect("Failed to free the RAM");

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

/// Searches the DTB for an SDHCI-compatible microSD controller (e.g.
/// Raspberry Pi 4's EMMC2, `compatible = "brcm,bcm2711-emmc2"`) and, if
/// found and operational, initializes it and probes for a card.
fn init_sdhci(dtb: &dtb::Dtb) -> Option<drivers::sdhci::Sdhci> {
    const SDHCI_COMPATIBLE_LIST: &[&[u8]] =
        &[b"brcm,bcm2711-emmc2", b"brcm,sdhci-brcmstb", b"generic-sdhci"];
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

/// Default MAC address used when the physical Virtio-Net device does not
/// support VIRTIO_NET_F_MAC (locally administered, QEMU/virtual convention).
const DEFAULT_MAC_ADDRESS: [u8; 6] = [0x52, 0x54, 0x00, 0x12, 0x34, 0x56];

/// Searches the DTB for a `virtio,mmio` node backed by a Virtio-Net device,
/// initializes the physical driver and returns it together with its GIC
/// interrupt id (if any) and the MAC address to expose to the guest.
fn init_virtio_net(dtb: &dtb::Dtb) -> (Option<virtio_net::VirtioNet>, Option<u32>, [u8; 6]) {
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
                    let mac = net.get_mac_address();
                    let mac = if mac == [0u8; 6] { DEFAULT_MAC_ADDRESS } else { mac };
                    return (Some(net), int_id, mac);
                }
            }
            None => {
                println!("Virtio-Net device is not present.");
                return (None, None, DEFAULT_MAC_ADDRESS);
            }
        }
    }
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
fn handle_net_rx() {
    let mut buffer = [0u8; drivers::virtio_net::VIRTIO_NET_RX_BUFFER_SIZE];
    let mut net = VIRTIO_NET.lock();
    let Some(net) = net.as_mut() else {
        return;
    };
    while let Some(length) = net.poll_rx(&mut buffer) {
        vm::input_net_packet(&buffer[..length]);
    }
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
            .allocate(layout.size(), layout.align())
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
                psci::spin_table_cpu_on(release_address, stack_address as u64);
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
        DEFAULT_MAC_ADDRESS,
    );
    vm::boot_vm(boot_address, argument)
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
