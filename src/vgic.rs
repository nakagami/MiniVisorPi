//!
//! Virtual Generic Interrupt Controller
//!

use crate::drivers::gicv2;
use crate::drivers::gicv2::{GicDistributor, GicGroup, GicHypervisorInterface};
use crate::mmio::gicv2::INJECT_INTERRUPT_INT_ID;
use crate::vm;

pub const MAINTENANCE_INTERRUPT_INTID: u32 = 25;

/* Fields (32-bit) of the GICv2 GICH_LR (List Register) */
const GICH_LR_VIRTUAL_ID: u32 = (1 << 10) - 1;
const GICH_LR_PHYSICAL_ID_OFFSET: u32 = 10;
const GICH_LR_PHYSICAL_ID: u32 = ((1 << 10) - 1) << GICH_LR_PHYSICAL_ID_OFFSET;
const GICH_LR_EOI_OFFSET: u32 = 19;
const GICH_LR_EOI: u32 = 1 << GICH_LR_EOI_OFFSET;
const GICH_LR_PRIORITY_OFFSET: u32 = 23;
const GICH_LR_STATE_OFFSET: u32 = 28;
const GICH_LR_STATE: u32 = 0b11 << GICH_LR_STATE_OFFSET;
const GICH_LR_STATE_INACTIVE: u32 = 0b00 << GICH_LR_STATE_OFFSET;
const GICH_LR_STATE_PENDING: u32 = 0b01 << GICH_LR_STATE_OFFSET;
const GICH_LR_GROUP1_OFFSET: u32 = 30;
const GICH_LR_HW_OFFSET: u32 = 31;
const GICH_LR_HW: u32 = 1 << GICH_LR_HW_OFFSET;

/* Maximum number of List Registers used by this hypervisor */
pub const NUMBER_OF_SUPPORTED_LRS: usize = 4;

pub fn init_vgic(gich: &GicHypervisorInterface, distributor: &GicDistributor) {
    gich.init();

    /* Enable Maintenance Interrupt */
    distributor.set_group(MAINTENANCE_INTERRUPT_INTID, GicGroup::NonSecureGroup1);
    distributor.set_priority(MAINTENANCE_INTERRUPT_INTID, 0x00);
    distributor.set_trigger_mode(MAINTENANCE_INTERRUPT_INTID, true);
    distributor.set_enable(MAINTENANCE_INTERRUPT_INTID, true);

    /* Enable INJECT_INTERRUPT_INT_ID */
    distributor.set_group(INJECT_INTERRUPT_INT_ID, GicGroup::NonSecureGroup1);
    distributor.set_priority(INJECT_INTERRUPT_INT_ID, 0x00);
    distributor.set_trigger_mode(INJECT_INTERRUPT_INT_ID, false);
    distributor.set_enable(INJECT_INTERRUPT_INT_ID, true);
}

pub fn create_list_register_entry(
    int_id: u32,
    group: u32,
    priority: u32,
    physical_int_id: Option<u32>,
) -> u32 {
    let mut entry = GICH_LR_STATE_PENDING
        | (group << GICH_LR_GROUP1_OFFSET)
        | (((priority >> 3) & 0x1F) << GICH_LR_PRIORITY_OFFSET)
        | (int_id & GICH_LR_VIRTUAL_ID);
    if let Some(p_int_id) = physical_int_id {
        entry |= GICH_LR_HW | ((p_int_id << GICH_LR_PHYSICAL_ID_OFFSET) & GICH_LR_PHYSICAL_ID);
    } else {
        entry |= GICH_LR_EOI;
    }
    entry
}

/// Injects `entry` into a free List Register of the current pCPU.
///
/// Returns `false` when every List Register is already occupied by an unrelated interrupt.
/// The caller must treat that as "not delivered": in particular, if the entry carried the
/// HW bit (so that the guest's deactivation of the virtual INTID would also deactivate the
/// physical one), the caller has to deactivate the physical interrupt itself, otherwise it
/// stays active forever and that pCPU never takes it again.
#[must_use]
pub fn add_virtual_interrupt(entry: u32) -> bool {
    let number_of_lrn = gicv2::get_gich_vtr_list_regs();
    let supported_lrn = number_of_lrn.min(NUMBER_OF_SUPPORTED_LRS);

    /* First pass: if this INTID already has an entry in some LR (pending, active, or
     * both), it must be merged into that same LR rather than allocated a fresh one.
     * A free LR at a lower index must NOT shadow a later LR already holding this INTID,
     * otherwise the same virtual interrupt ends up duplicated across two LRs, wasting a
     * List Register and eventually starving unrelated interrupts (e.g. the virtual timer)
     * once all LRs fill up with such duplicates. */
    for i in 0..supported_lrn {
        let gich_lrn = gicv2::get_gich_lr(i);
        if (gich_lrn & GICH_LR_STATE) != GICH_LR_STATE_INACTIVE
            && (gich_lrn & GICH_LR_VIRTUAL_ID) == (entry & GICH_LR_VIRTUAL_ID)
        {
            gicv2::set_gich_lr(i, gich_lrn | GICH_LR_STATE_PENDING);
            return true;
        }
    }

    /* Second pass: no existing entry for this INTID, so claim the first free LR. */
    for i in 0..supported_lrn {
        let gich_lrn = gicv2::get_gich_lr(i);
        if (gich_lrn & GICH_LR_STATE) == GICH_LR_STATE_INACTIVE {
            gicv2::set_gich_lr(i, entry);
            return true;
        }
    }

    println!("GICH_LR is overflowed.");
    false
}

pub fn maintenance_interrupt_handler() {
    let number_of_lrn = gicv2::get_gich_vtr_list_regs();
    let supported_lrn = number_of_lrn.min(NUMBER_OF_SUPPORTED_LRS);
    let mut eoi_bits = gicv2::get_gich_eisr();

    for i in 0..supported_lrn {
        if (eoi_bits & 1) != 0 {
            let entry = gicv2::get_gich_lr(i);
            let int_id = entry & GICH_LR_VIRTUAL_ID;
            let vm = vm::get_current_vm();
            let mut distributor = vm.get_gic_distributor_mmio().lock();
            distributor.change_pending_status(int_id, false);
            distributor.change_active_status(int_id, false);
            gicv2::set_gich_lr(i, 0);
        }
        eoi_bits >>= 1;
    }
}
