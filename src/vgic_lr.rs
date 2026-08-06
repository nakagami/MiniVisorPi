//!
//! Pure GICv2 GICH_LR (List Register) bit-packing logic.
//!
//! Kept in its own module, free of any hardware/MMIO access, so this
//! logic can be exercised by host-side unit tests (see
//! `allocator_tests/src/lib.rs`) without needing to stub out the rest of
//! the hypervisor's aarch64-specific drivers.
//!

/* Fields (32-bit) of the GICv2 GICH_LR (List Register) */
const GICH_LR_VIRTUAL_ID: u32 = (1 << 10) - 1;
const GICH_LR_PHYSICAL_ID_OFFSET: u32 = 10;
const GICH_LR_PHYSICAL_ID: u32 = ((1 << 10) - 1) << GICH_LR_PHYSICAL_ID_OFFSET;
const GICH_LR_EOI_OFFSET: u32 = 19;
const GICH_LR_EOI: u32 = 1 << GICH_LR_EOI_OFFSET;
const GICH_LR_PRIORITY_OFFSET: u32 = 23;
const GICH_LR_STATE_PENDING: u32 = 0b01 << 28;
const GICH_LR_GROUP1_OFFSET: u32 = 30;
const GICH_LR_HW_OFFSET: u32 = 31;
const GICH_LR_HW: u32 = 1 << GICH_LR_HW_OFFSET;

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

#[cfg(test)]
mod tests {
    use super::*;

    const GICH_LR_STATE: u32 = 0b11 << 28;
    const GICH_LR_STATE_PENDING_MASK: u32 = 0b01 << 28;

    #[test]
    fn software_interrupt_sets_eoi_and_pending_state_but_not_hw_bit() {
        let entry = create_list_register_entry(42, 1, 0xA0, None);
        assert_eq!(entry & GICH_LR_VIRTUAL_ID, 42);
        assert_eq!(entry & GICH_LR_GROUP1_MASK, GICH_LR_GROUP1_MASK);
        assert_eq!(entry & GICH_LR_STATE, GICH_LR_STATE_PENDING_MASK);
        assert_eq!(entry & GICH_LR_HW, 0);
        assert_eq!(entry & GICH_LR_EOI, GICH_LR_EOI);
    }

    #[test]
    fn hardware_interrupt_sets_hw_bit_and_physical_id_but_not_eoi() {
        let entry = create_list_register_entry(30, 0, 0x80, Some(27));
        assert_eq!(entry & GICH_LR_VIRTUAL_ID, 30);
        assert_eq!(entry & GICH_LR_GROUP1_MASK, 0);
        assert_eq!(entry & GICH_LR_HW, GICH_LR_HW);
        assert_eq!(entry & GICH_LR_EOI, 0);
        assert_eq!(
            (entry & GICH_LR_PHYSICAL_ID) >> GICH_LR_PHYSICAL_ID_OFFSET,
            27
        );
    }

    #[test]
    fn priority_is_truncated_to_its_top_5_bits_at_the_correct_offset() {
        let entry = create_list_register_entry(1, 0, 0xFF, None);
        assert_eq!((entry >> GICH_LR_PRIORITY_OFFSET) & 0x1F, 0x1F);
    }

    #[test]
    fn int_id_is_masked_to_10_bits() {
        let entry = create_list_register_entry(0xFFFF_FFFF, 0, 0, None);
        assert_eq!(entry & GICH_LR_VIRTUAL_ID, GICH_LR_VIRTUAL_ID);
    }

    const GICH_LR_GROUP1_MASK: u32 = 1 << GICH_LR_GROUP1_OFFSET;
}
