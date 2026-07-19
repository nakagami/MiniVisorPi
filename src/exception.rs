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
    b   synchronous_current_el_stack_pointer_x

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
);

pub fn setup_exception() {
    unsafe extern "C" {
        static exception_table: *const u8;
    }
    unsafe { asm::set_vbar_el2(&exception_table as *const _ as usize as u64) };
}

extern "C" fn synchronous_handler(registers: *mut Registers) {
    let esr_el2 = asm::get_esr_el2();
    let ec = esr_el2 & ESR_EL2_EC;
    match ec {
        ESR_EL2_EC_DATA_ABORT => data_abort_handler(unsafe { &mut *registers }, esr_el2),
        _ => {
            panic!("Unknown Exception: {}", ec >> ESR_EL2_EC_BITS_OFFSET);
        }
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
        crate::handle_input(&crate::PL011_DEVICE);
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
        vm::get_current_vm()
            .handle_mmio_write(address as usize, access_width, register_value)
            .expect("Failed to handle MMIO");
    } else {
        *register = vm::get_current_vm()
            .handle_mmio_read(address as usize, access_width)
            .expect("Failed to handle MMIO");
    }

    unsafe { asm::advance_elr_el2() };
}

extern "C" fn irq_handler() {
    let (interrupt_number, group) = GicCpuInterface::get_acknowledge();
    let mut deactivate = true;
    if interrupt_number == unsafe { crate::PL011_INT_ID } {
        crate::handle_input(&crate::PL011_DEVICE);
    } else if interrupt_number == vgic::MAINTENANCE_INTERRUPT_INTID {
        vgic::maintenance_interrupt_handler();
    } else if interrupt_number == gicv2::INJECT_INTERRUPT_INT_ID {
        gicv2::inject_interrupt_handler();
    } else if interrupt_number == unsafe { generic_timer::GENERIC_TIMER_PHYSICAL_INT_ID } {
        generic_timer::generic_timer_interrupt_handler();
        deactivate = false; /* Deactivate is handled by the VGIC */
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
