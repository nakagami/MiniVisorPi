//!
//! Interrupt control
//!
use crate::asm;
use crate::drivers::{generic_timer, gicv2::*};
use crate::mmio::gicv2;
use crate::registers::*;
use crate::vgic;
use crate::vm;

use core::arch::global_asm;

#[repr(C)]
pub struct Registers {
    pub x0: u64,
    pub x1: u64,
    pub x2: u64,
    pub x3: u64,
    pub x4: u64,
    pub x5: u64,
    pub x6: u64,
    pub x7: u64,
    pub x8: u64,
    pub x9: u64,
    pub x10: u64,
    pub x11: u64,
    pub x12: u64,
    pub x13: u64,
    pub x14: u64,
    pub x15: u64,
    pub x16: u64,
    pub x17: u64,
    pub x18: u64,
    pub x19: u64,
    pub x20: u64,
    pub x21: u64,
    pub x22: u64,
    pub x23: u64,
    pub x24: u64,
    pub x25: u64,
    pub x26: u64,
    pub x27: u64,
    pub x28: u64,
    pub x29: u64,
    pub x30: u64,
    padding: u64,
}

impl Registers {
    /// Number of `u64` slots in this struct (x0-x30 plus the trailing padding
    /// slot that `exception_table`'s `stp x30, xzr, ...` always zeroes).
    pub const NUMBER_OF_SLOTS: usize = 32;

    /// Snapshots this trap frame as a flat array, for saving into a VCPU's
    /// off-CPU context (see `vm::VcpuContext`) when it yields the pCPU.
    pub fn as_array(&self) -> [u64; Self::NUMBER_OF_SLOTS] {
        unsafe { core::mem::transmute_copy(self) }
    }

    /// Overwrites this trap frame with a previously saved VCPU context, so
    /// that the shared `exit_exception` epilogue (which always pops
    /// GPRs from *this* trap frame) resumes the restored VCPU instead of
    /// the one that originally took the trap.
    pub fn load_array(&mut self, array: &[u64; Self::NUMBER_OF_SLOTS]) {
        unsafe { core::ptr::write(self as *mut Self as *mut [u64; Self::NUMBER_OF_SLOTS], *array) };
    }
}

/* Exception table */
global_asm!(
    "
.section .text
.balign 0x800
.size   exception_table, 0x800
.global exception_table
exception_table:

.balign 0x080
synchronous_current_el_stack_pointer_0:
    b   synchronous_current_el_stack_pointer_0

.balign 0x080
irq_current_el_stack_pointer_0:
    b   irq_current_el_stack_pointer_0

.balign 0x080
fiq_current_el_stack_pointer_0:
    b   fiq_current_el_stack_pointer_0

.balign 0x080
s_error_current_el_stack_pointer_0:
    b   s_error_current_el_stack_pointer_0

.balign 0x080
synchronous_current_el_stack_pointer_x:
    sub sp,   sp, #(8 * 32)
    stp x30, xzr, [sp, #( 15 * 16)]
    stp x28, x29, [sp, #( 14 * 16)]
    stp x26, x27, [sp, #( 13 * 16)]
    stp x24, x25, [sp, #( 12 * 16)]
    stp x22, x23, [sp, #( 11 * 16)]
    stp x20, x21, [sp, #( 10 * 16)]
    stp x18, x19, [sp, #(  9 * 16)]
    stp x16, x17, [sp, #(  8 * 16)]
    stp x14, x15, [sp, #(  7 * 16)]
    stp x12, x13, [sp, #(  6 * 16)]
    stp x10, x11, [sp, #(  5 * 16)]
    stp  x8,  x9, [sp, #(  4 * 16)]
    stp  x6,  x7, [sp, #(  3 * 16)]
    stp  x4,  x5, [sp, #(  2 * 16)]
    stp  x2,  x3, [sp, #(  1 * 16)]
    stp  x0,  x1, [sp, #(  0 * 16)]
    mov  x0, sp
    b   {el2_synchronous_handler}

.balign 0x080
irq_current_el_stack_pointer_x:
    sub sp,   sp, #(8 * 32)
    stp x30, xzr, [sp, #( 15 * 16)]
    stp x28, x29, [sp, #( 14 * 16)]
    stp x26, x27, [sp, #( 13 * 16)]
    stp x24, x25, [sp, #( 12 * 16)]
    stp x22, x23, [sp, #( 11 * 16)]
    stp x20, x21, [sp, #( 10 * 16)]
    stp x18, x19, [sp, #(  9 * 16)]
    stp x16, x17, [sp, #(  8 * 16)]
    stp x14, x15, [sp, #(  7 * 16)]
    stp x12, x13, [sp, #(  6 * 16)]
    stp x10, x11, [sp, #(  5 * 16)]
    stp  x8,  x9, [sp, #(  4 * 16)]
    stp  x6,  x7, [sp, #(  3 * 16)]
    stp  x4,  x5, [sp, #(  2 * 16)]
    stp  x2,  x3, [sp, #(  1 * 16)]
    stp  x0,  x1, [sp, #(  0 * 16)]
    mov  x0, sp
    adr x30, exit_exception
    b   {irq_handler}

.balign 0x080
fiq_current_el_stack_pointer_x:
    sub sp,   sp, #(8 * 32)
    stp x30, xzr, [sp, #( 15 * 16)]
    stp x28, x29, [sp, #( 14 * 16)]
    stp x26, x27, [sp, #( 13 * 16)]
    stp x24, x25, [sp, #( 12 * 16)]
    stp x22, x23, [sp, #( 11 * 16)]
    stp x20, x21, [sp, #( 10 * 16)]
    stp x18, x19, [sp, #(  9 * 16)]
    stp x16, x17, [sp, #(  8 * 16)]
    stp x14, x15, [sp, #(  7 * 16)]
    stp x12, x13, [sp, #(  6 * 16)]
    stp x10, x11, [sp, #(  5 * 16)]
    stp  x8,  x9, [sp, #(  4 * 16)]
    stp  x6,  x7, [sp, #(  3 * 16)]
    stp  x4,  x5, [sp, #(  2 * 16)]
    stp  x2,  x3, [sp, #(  1 * 16)]
    stp  x0,  x1, [sp, #(  0 * 16)]
    mov  x0, sp
    adr x30, exit_exception
    b   {fiq_handler}

.balign 0x080
s_error_current_el_stack_pointer_x:
    b   s_error_current_el_stack_pointer_x

.balign 0x080
synchronous_lower_el_aarch64:
    sub sp,   sp, #(8 * 32)
    stp x30, xzr, [sp, #( 15 * 16)]
    stp x28, x29, [sp, #( 14 * 16)]
    stp x26, x27, [sp, #( 13 * 16)]
    stp x24, x25, [sp, #( 12 * 16)]
    stp x22, x23, [sp, #( 11 * 16)]
    stp x20, x21, [sp, #( 10 * 16)]
    stp x18, x19, [sp, #(  9 * 16)]
    stp x16, x17, [sp, #(  8 * 16)]
    stp x14, x15, [sp, #(  7 * 16)]
    stp x12, x13, [sp, #(  6 * 16)]
    stp x10, x11, [sp, #(  5 * 16)]
    stp  x8,  x9, [sp, #(  4 * 16)]
    stp  x6,  x7, [sp, #(  3 * 16)]
    stp  x4,  x5, [sp, #(  2 * 16)]
    stp  x2,  x3, [sp, #(  1 * 16)]
    stp  x0,  x1, [sp, #(  0 * 16)]
    mov  x0, sp
    adr x30, exit_exception
    b   {synchronous_handler}

.balign 0x080
irq_lower_el_aarch64:
    sub sp,   sp, #(8 * 32)
    stp x30, xzr, [sp, #( 15 * 16)]
    stp x28, x29, [sp, #( 14 * 16)]
    stp x26, x27, [sp, #( 13 * 16)]
    stp x24, x25, [sp, #( 12 * 16)]
    stp x22, x23, [sp, #( 11 * 16)]
    stp x20, x21, [sp, #( 10 * 16)]
    stp x18, x19, [sp, #(  9 * 16)]
    stp x16, x17, [sp, #(  8 * 16)]
    stp x14, x15, [sp, #(  7 * 16)]
    stp x12, x13, [sp, #(  6 * 16)]
    stp x10, x11, [sp, #(  5 * 16)]
    stp  x8,  x9, [sp, #(  4 * 16)]
    stp  x6,  x7, [sp, #(  3 * 16)]
    stp  x4,  x5, [sp, #(  2 * 16)]
    stp  x2,  x3, [sp, #(  1 * 16)]
    stp  x0,  x1, [sp, #(  0 * 16)]
    mov  x0, sp
    adr x30, exit_exception
    b   {irq_handler}

.balign 0x080
fiq_lower_el_aarch64:
    sub sp,   sp, #(8 * 32)
    stp x30, xzr, [sp, #( 15 * 16)]
    stp x28, x29, [sp, #( 14 * 16)]
    stp x26, x27, [sp, #( 13 * 16)]
    stp x24, x25, [sp, #( 12 * 16)]
    stp x22, x23, [sp, #( 11 * 16)]
    stp x20, x21, [sp, #( 10 * 16)]
    stp x18, x19, [sp, #(  9 * 16)]
    stp x16, x17, [sp, #(  8 * 16)]
    stp x14, x15, [sp, #(  7 * 16)]
    stp x12, x13, [sp, #(  6 * 16)]
    stp x10, x11, [sp, #(  5 * 16)]
    stp  x8,  x9, [sp, #(  4 * 16)]
    stp  x6,  x7, [sp, #(  3 * 16)]
    stp  x4,  x5, [sp, #(  2 * 16)]
    stp  x2,  x3, [sp, #(  1 * 16)]
    stp  x0,  x1, [sp, #(  0 * 16)]
    mov  x0, sp
    adr x30, exit_exception
    b   {fiq_handler}

.balign 0x080
s_error_lower_el_aarch64:
    b   s_error_lower_el_aarch64

.balign 0x080
synchronous_lower_el_aarch32:
    b   synchronous_lower_el_aarch32

.balign 0x080
irq_lower_el_aarch32:
    b   irq_lower_el_aarch32

.balign 0x080
fiq_lower_el_aarch32:
    b   fiq_lower_el_aarch32

.balign 0x080
s_error_lower_el_aarch32:
    b   s_error_lower_el_aarch32

exit_exception:
    ldp x30, xzr, [sp, #( 15 * 16)]
    ldp x28, x29, [sp, #( 14 * 16)]
    ldp x26, x27, [sp, #( 13 * 16)]
    ldp x24, x25, [sp, #( 12 * 16)]
    ldp x22, x23, [sp, #( 11 * 16)]
    ldp x20, x21, [sp, #( 10 * 16)]
    ldp x18, x19, [sp, #(  9 * 16)]
    ldp x16, x17, [sp, #(  8 * 16)]
    ldp x14, x15, [sp, #(  7 * 16)]
    ldp x12, x13, [sp, #(  6 * 16)]
    ldp x10, x11, [sp, #(  5 * 16)]
    ldp  x8,  x9, [sp, #(  4 * 16)]
    ldp  x6,  x7, [sp, #(  3 * 16)]
    ldp  x4,  x5, [sp, #(  2 * 16)]
    ldp  x2,  x3, [sp, #(  1 * 16)]
    ldp  x0,  x1, [sp, #(  0 * 16)]
    add  sp,  sp, #(8 * 32)
    eret
",
irq_handler = sym irq_handler,
synchronous_handler = sym synchronous_handler,
fiq_handler = sym fiq_handler,
el2_synchronous_handler = sym el2_synchronous_handler,
);

pub fn setup_exception() {
    unsafe extern "C" {
        static exception_table: *const u8;
    }
    unsafe { asm::set_vbar_el2(&exception_table as *const _ as usize as u64) };
}

/// Handles a synchronous exception taken to EL2 while already running at
/// EL2 (current EL, SP_ELx), i.e. a fault caused directly by the
/// hypervisor's own code (e.g. an unmapped/invalid MMIO or memory access),
/// as opposed to a trapped lower-EL (guest) exception. Previously this
/// vector was an infinite self-branch, which silently hung with no
/// diagnostic output at all whenever the hypervisor itself faulted; report
/// the fault syndrome instead so the failure is visible.
extern "C" fn el2_synchronous_handler(registers: *mut Registers) -> ! {
    let esr_el2 = asm::get_esr_el2();
    let far_el2 = asm::get_far_el2();
    let elr_el2 = asm::get_elr_el2();
    let ec = (esr_el2 & ESR_EL2_EC) >> ESR_EL2_EC_BITS_OFFSET;
    let regs = unsafe { &*registers };
    panic!(
        "Unhandled EL2-native synchronous exception: ec={ec:#X} esr_el2={esr_el2:#X} \
         far_el2={far_el2:#X} elr_el2={elr_el2:#X} x0={:#X} x1={:#X} x2={:#X} x30(caller)={:#X}",
        regs.x0, regs.x1, regs.x2, regs.x30
    );
}

extern "C" fn synchronous_handler(registers: *mut Registers) {
    let esr_el2 = asm::get_esr_el2();
    let ec = esr_el2 & ESR_EL2_EC;
    match ec {
        ESR_EL2_EC_DATA_ABORT => data_abort_handler(unsafe { &mut *registers }, esr_el2),
        ESR_EL2_EC_WFX => wfx_handler(unsafe { &mut *registers }),
        ESR_EL2_EC_SMC64 => crate::handle_guest_smc(unsafe { &mut *registers }),
        _ => {
            let far_el2 = asm::get_far_el2();
            let elr_el2 = asm::get_elr_el2();
            let mpidr = asm::get_mpidr_el1();
            panic!(
                "Unknown Exception: {} esr_el2={esr_el2:#X} far_el2={far_el2:#X} \
                 elr_el2={elr_el2:#X} mpidr_el1={mpidr:#X}",
                ec >> ESR_EL2_EC_BITS_OFFSET
            );
        }
    }
}

/// Handles a guest WFI/WFE trapped to EL2 via HCR_EL2.TWI. Since physical device
/// interrupts may not always reach this Non-secure EL2 hypervisor as expected on
/// real Raspberry Pi 4 hardware, WFI is used as a reliable polling point:
/// 1) drain physical UART RX and inject to the guest PL011
/// 2) when using the GENET backend, poll physical RX and inject to guest virtio-net
/// 3) if another VCPU is queued on this same physical CPU, cooperatively switch to it
///    (see `vm::try_yield_to_next_vcpu`) instead of always resuming the caller.
/// Otherwise (or if there is nothing else to run), just advance past WFI so the
/// guest re-evaluates its wait condition.
/// Diagnostic: how often the WFI polling path has run (see the `stat`
/// console command).
pub static WFI_POLL_COUNT: core::sync::atomic::AtomicU64 =
    core::sync::atomic::AtomicU64::new(0);

/// Per-pCPU CNTPCT_EL0 timestamp of the last WFI-triggered device poll.
static LAST_WFI_POLL: [core::sync::atomic::AtomicU64; 8] =
    [const { core::sync::atomic::AtomicU64::new(0) }; 8];

fn wfx_handler(registers: &mut Registers) {
    WFI_POLL_COUNT.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    /* Rate-limit the UART/net polling (and guest-console flush) to once per
     * ~1ms per pCPU. Idle vCPUs hit WFI in a tight loop, and polling on every
     * trap made the global UART/serial locks (PL011_DEVICE, UART_INPUT_LOCK)
     * a permanent convoy: measurements during boot showed ~1.2M polls with
     * each polling pCPU queueing ahead of any vCPU trying to do real console
     * I/O, inflating per-character output cost to ~3ms. On QEMU the physical
     * UART IRQ delivers input independently of this poll, and on real Pi4
     * hardware a 1ms worst-case input latency is unnoticeable on a console. */
    let cpu = (crate::asm::get_mpidr_el1() & 0xFF) as usize;
    if let Some(slot) = LAST_WFI_POLL.get(cpu) {
        let now = crate::asm::get_cntpct_el0();
        let last = slot.load(core::sync::atomic::Ordering::Relaxed);
        if now.wrapping_sub(last) >= crate::asm::get_cntfrq_el0() / 1000 {
            slot.store(now, core::sync::atomic::Ordering::Relaxed);
            crate::handle_uart_interrupt();
            if crate::needs_net_polling_on_wfx() {
                crate::handle_net_rx();
            }
            /* Flush guest console output buffered by the emulated PL011 (see
             * mmio::pl011): a guest waiting for input idles via WFI, so its
             * prompt appears within ~1ms without an explicit newline. */
            vm::get_current_vm().get_pl011_mmio().lock().flush_tx();
        }
    }
    if !vm::try_yield_to_next_vcpu(registers) {
        unsafe { asm::advance_elr_el2() };
    }
}

/// Handles a physical FIQ, i.e. a Group 0 (Secure) interrupt. On real hardware, an SPI
/// that no secure-world firmware ever explicitly assigned to Non-secure Group 1 stays in
/// Group 0 (see the `GICC_AIAR` doc comment in drivers/gicv2.rs), so this is the actual
/// delivery path for e.g. the PL011's physical interrupt on real Raspberry Pi 4 hardware,
/// even though this driver's own `set_group` call requests Group 1. Dispatches the same
/// way `irq_handler` does, since from this hypervisor's perspective a Group 0 interrupt
/// routed here still just means "a device this hypervisor owns wants attention".
extern "C" fn fiq_handler() {
    let interrupt_number = GicCpuInterface::get_acknowledge_group0();
    if interrupt_number == unsafe { crate::PL011_INT_ID } {
        crate::handle_uart_interrupt();
    } else if interrupt_number != GicCpuInterface::SPURIOUS_INT_ID {
        println!("Unhandled physical FIQ (Group 0): {interrupt_number}");
    }
    if interrupt_number != GicCpuInterface::SPURIOUS_INT_ID {
        GicCpuInterface::eoi_group0(interrupt_number);
    }
}

fn data_abort_handler(registers: &mut Registers, esr_el2: u64) {
    if esr_el2 & ESR_EL2_ISS_ISV == 0 {
        panic!("Data Abort Info is not available.");
    }
    let is_64bit_register = (esr_el2 & ESR_EL2_ISS_SF) != 0;
    let access_width = match (esr_el2 & ESR_EL2_ISS_SAS) >> ESR_EL2_ISS_SAS_BITS_OFFSET {
        0b00 => 8,
        0b01 => 16,
        0b10 => 32,
        0b11 => 64,
        _ => unreachable!(),
    };
    let is_write_access = (esr_el2 & ESR_EL2_ISS_WNR) != 0;

    let register_number = ((esr_el2 & ESR_EL2_ISS_SRT) >> ESR_EL2_ISS_SRT_BITS_OFFSET) as usize;
    let register: &mut u64 =
        &mut unsafe { &mut *(registers as *mut _ as usize as *mut [u64; 32]) }[register_number];

    let address = (((asm::get_hpfar_el2() & HPFAR_EL2_FIPA) >> HPFAR_EL2_FIPA_BITS_OFFSET)
        << crate::paging::PAGE_SHIFT)
        | (asm::get_far_el2() & ((1 << crate::paging::PAGE_SHIFT) - 1));

    if is_write_access {
        let register_value = if is_64bit_register {
            *register
        } else {
            *register & (u32::MAX as u64)
        };
        if let Err(()) = vm::get_current_vm().handle_mmio_write(
            address as usize,
            access_width,
            register_value,
        ) {
            panic!(
                "Failed to handle MMIO write: address={address:#X} \
                 access_width={access_width} value={register_value:#X}"
            );
        }
    } else {
        match vm::get_current_vm().handle_mmio_read(address as usize, access_width) {
            Ok(value) => *register = value,
            Err(()) => panic!(
                "Failed to handle MMIO read: address={address:#X} access_width={access_width}"
            ),
        }
    }

    unsafe { asm::advance_elr_el2() };
}

extern "C" fn irq_handler() {
    let (interrupt_number, group) = GicCpuInterface::get_acknowledge();
    let mut deactivate = true;
    if interrupt_number == unsafe { crate::PL011_INT_ID } {
        crate::handle_uart_interrupt();
    } else if interrupt_number == vgic::MAINTENANCE_INTERRUPT_INTID {
        vgic::maintenance_interrupt_handler();
    } else if interrupt_number == gicv2::INJECT_INTERRUPT_INT_ID {
        gicv2::inject_interrupt_handler();
    } else if interrupt_number == unsafe { generic_timer::GENERIC_TIMER_PHYSICAL_INT_ID } {
        /* When the virtual timer interrupt was injected, its List Register carries the HW
         * bit plus this physical INTID, so the guest's own deactivation of the virtual
         * interrupt deactivates the physical one and the hypervisor must not do it here.
         *
         * When injection did *not* happen (the guest currently has this banked PPI disabled
         * -- e.g. it is inside `gic_cpu_init` on this very CPU -- or every List Register was
         * full), nothing will ever deactivate the physical PPI. Leaving it active would mask
         * it in the physical GIC forever, permanently killing this pCPU's timer tick (seen
         * from the guest as an RCU stall with no further scheduler ticks on that CPU), so
         * deactivate it here instead. The timer condition is level-triggered and still met,
         * so the line simply re-asserts and the tick is retried a moment later. */
        deactivate = !generic_timer::generic_timer_interrupt_handler();
    } else if interrupt_number == unsafe { crate::VIRTIO_NET_INT_ID } {
        crate::handle_net_rx();
    } else if interrupt_number != GicCpuInterface::SPURIOUS_INT_ID {
        /* An interrupt fired that isn't recognized by any branch above (e.g.
         * a mismatch between the physical SPI number actually wired to a
         * device and the ID this driver computed/registered for it). Report
         * it instead of silently EOI/deactivating it, since otherwise the
         * device's events go unnoticed with no diagnostic at all. */
        println!("Unhandled physical interrupt: {interrupt_number}");
    }
    /* GICv2 requires software not to perform a priority drop/deactivation
     * for the spurious ID (1023): there is no corresponding pending
     * interrupt to drop/deactivate, so software must simply ignore it. */
    if interrupt_number != GicCpuInterface::SPURIOUS_INT_ID {
        GicCpuInterface::drop_priority(interrupt_number, group);
        if deactivate {
            GicCpuInterface::deactivate(interrupt_number);
        }
    }
}
