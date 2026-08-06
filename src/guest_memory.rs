//!
//! Pure guest virtual→physical RAM address translation.
//!
//! Kept in its own module, free of any hardware access or `alloc`
//! dependencies, so this logic can be exercised by host-side unit tests
//! (see `allocator_tests/src/lib.rs`) without needing to stub out the rest
//! of `VM`'s aarch64-specific state (Stage 2 tables, banked EL1 system
//! registers, MMIO handlers, etc).
//!

/// Translates `virtual_address` (an address within the guest's own identity
/// or kernel virtual mapping of its RAM) into the corresponding
/// hypervisor-physical address, given that guest's RAM window
/// (`ram_virtual_base_address`, `ram_physical_base_address`, `ram_size`).
///
/// Returns `None` when `virtual_address` falls outside that guest's RAM
/// window (e.g. it points at an MMIO region instead), mirroring
/// `VM::get_physical_address`, which callers must not conflate with a
/// hypervisor-physical address that happens to be `0`.
pub fn translate_guest_physical_address(
    virtual_address: usize,
    ram_virtual_base_address: usize,
    ram_physical_base_address: usize,
    ram_size: usize,
) -> Option<usize> {
    if (ram_virtual_base_address..(ram_virtual_base_address + ram_size))
        .contains(&virtual_address)
    {
        Some(virtual_address - ram_virtual_base_address + ram_physical_base_address)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn address_at_ram_start_maps_to_physical_base() {
        assert_eq!(
            translate_guest_physical_address(0x4008_0000, 0x4008_0000, 0x1000_0000, 0x1000),
            Some(0x1000_0000)
        );
    }

    #[test]
    fn address_within_ram_window_is_offset_correctly() {
        assert_eq!(
            translate_guest_physical_address(0x4008_1234, 0x4008_0000, 0x1000_0000, 0x1_0000),
            Some(0x1000_1234)
        );
    }

    #[test]
    fn address_at_the_last_valid_byte_is_still_in_range() {
        assert_eq!(
            translate_guest_physical_address(0x4008_0FFF, 0x4008_0000, 0x1000_0000, 0x1000),
            Some(0x1000_0FFF)
        );
    }

    #[test]
    fn address_at_ram_end_is_out_of_range() {
        // ram_size = 0x1000 means valid addresses are [base, base + 0x1000);
        // the address exactly at the end is one byte past the last valid one.
        assert_eq!(
            translate_guest_physical_address(0x4008_1000, 0x4008_0000, 0x1000_0000, 0x1000),
            None
        );
    }

    #[test]
    fn address_below_ram_window_is_out_of_range() {
        assert_eq!(
            translate_guest_physical_address(0x1000, 0x4008_0000, 0x1000_0000, 0x1000),
            None
        );
    }
}
