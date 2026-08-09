//!
//! Virtual Generic Interrupt Controller
//!

use crate::drivers::gicv2;
use crate::drivers::gicv2::{GicDistributor, GicGroup, GicHypervisorInterface};
use crate::mmio::gicv2::INJECT_INTERRUPT_INT_ID;
use crate::vgic_lr;
use crate::vm;

use core::sync::atomic::{AtomicU64, Ordering};

pub const MAINTENANCE_INTERRUPT_INTID: u32 = 25;

/* Fields (32-bit) of the GICv2 GICH_LR (List Register) that this module still
 * needs to read back (see `add_virtual_interrupt`/`maintenance_interrupt_handler`
 * below); the bit-packing logic that *builds* an entry from scratch has been
 * moved to the pure, host-testable `vgic_lr` module. */
const GICH_LR_VIRTUAL_ID: u32 = (1 << 10) - 1;
const GICH_LR_STATE_OFFSET: u32 = 28;
const GICH_LR_STATE: u32 = 0b11 << GICH_LR_STATE_OFFSET;
const GICH_LR_STATE_INACTIVE: u32 = 0b00 << GICH_LR_STATE_OFFSET;
const GICH_LR_STATE_PENDING: u32 = 0b01 << GICH_LR_STATE_OFFSET;

/// Maximum number of List Registers used by this hypervisor.
pub const NUMBER_OF_SUPPORTED_LRS: usize = 4;

/// Diagnostic (see the `stat` console command): how many times
/// `add_virtual_interrupt` found every List Register occupied. Failed
/// injections are retried later (see [`maintenance_interrupt_handler`]),
/// so this is not by itself an error -- but a rapidly increasing count
/// under load means the guest consumes virtual interrupts more slowly
/// than they arrive. (This used to be a `println!` per failure, which
/// under a real overflow flood saturated the physical UART and amplified
/// the very interrupt latency that caused the overflow.)
pub static LR_OVERFLOW_COUNT: AtomicU64 = AtomicU64::new(0);

/// Re-exported so existing callers (e.g. `mmio::gicv2`) can keep referring to
/// `vgic::create_list_register_entry` unchanged; the implementation itself
/// lives in `vgic_lr` so it can be unit-tested on the host.
pub use vgic_lr::create_list_register_entry;

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

    LR_OVERFLOW_COUNT.fetch_add(1, Ordering::Relaxed);
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

    /* Every EOI processed above freed a List Register, so now retry the
     * injections that previously failed because all LRs were occupied
     * (LR overflow). This is the *only* recovery path for two classes of
     * otherwise-lost interrupts:
     *
     * 1. Cross-pCPU injections queued in `to_inject_interrupt` (guest
     *    IPIs/SGIs and SPIs targeting another pCPU): dropping one of
     *    these deadlocks the guest's SMP kernel (observed as a
     *    boot-time wedge under an LR-overflow flood).
     * 2. Same-pCPU injections whose INTID stayed pending in the virtual
     *    Distributor: edge-triggered virtual interrupts (virtio-net/blk/
     *    console) never re-assert once their notification was lost, so
     *    the guest would wait for them forever.
     *
     * The virtual Generic Timer PPI is deliberately *not* covered by
     * (2): its retries are driven by the level-triggered physical PPI
     * re-asserting (see `irq_handler`), which also keeps the LR HW bit
     * intact for guest-initiated physical deactivation. */
    crate::mmio::gicv2::inject_interrupt_handler();
    vm::get_current_vm()
        .get_gic_distributor_mmio()
        .lock()
        .inject_pending_for_current_cpu();
}
