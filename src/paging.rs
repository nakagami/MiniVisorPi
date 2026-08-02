//!
//! Stage 2 paging implementation
//!

use crate::allocate_pages;
use crate::asm;
use crate::registers::*;

use core::slice::from_raw_parts_mut;

#[derive(Clone)]
struct Descriptor(u64);

#[allow(dead_code)]
#[derive(Copy, Clone, Eq, PartialEq)]
#[allow(clippy::enum_variant_names)]
pub enum Shareability {
    NonShareable = 0b00,
    OuterShareable = 0b10,
    InnerShareable = 0b11,
}

pub const PAGE_SHIFT: usize = 12;
pub const PAGE_SIZE: usize = 1 << PAGE_SHIFT;

impl Descriptor {
    const TABLE_ADDRESS_MASK: u64 = ((1 << 50) - 1) & !(PAGE_SIZE as u64 - 1);
    const OUTPUT_ADDRESS_MASK: u64 = ((1 << 50) - 1) & !(PAGE_SIZE as u64 - 1);
    const AF_OFFSET: u64 = 10;
    const AF: u64 = 1 << Self::AF_OFFSET;
    const SH_OFFSET: u64 = 8;
    const SH: u64 = 0b11 << Self::SH_OFFSET;
    const S2AP_OFFSET: u64 = 6;
    const S2AP: u64 = 0b11 << Self::S2AP_OFFSET;
    const ATTR_INDEX_OFFSET: u64 = 2;
    const ATTR_INDEX: u64 = 0b1111 << Self::ATTR_INDEX_OFFSET;
    const ATTR_WRITE_BACK: u64 = 0b1111 << Self::ATTR_INDEX_OFFSET;

    const fn new() -> Self {
        Self(0)
    }

    fn init(&mut self) {
        *self = Self::new();
    }

    fn validate_as_page_descriptor(&mut self) {
        self.0 |= 0b11;
    }

    fn validate_as_table_descriptor(&mut self) {
        self.0 |= 0b11;
    }

    fn validate_as_block_descriptor(&mut self) {
        self.0 |= 0b01;
    }

    const fn is_table_descriptor(&self) -> bool {
        (self.0 & 0b11) == 0b11
    }

    const fn get_next_level_table_address(&self) -> usize {
        (self.0 & Self::TABLE_ADDRESS_MASK) as usize
    }

    fn set_output_address(&mut self, output_address: usize) {
        self.0 = (self.0 & !Self::OUTPUT_ADDRESS_MASK) | (output_address as u64) | Self::AF;
    }

    fn set_shareability(&mut self, shareability: Shareability) {
        self.0 = (self.0 & !Self::SH) | ((shareability as u64) << Self::SH_OFFSET);
    }

    fn set_permission(&mut self, permission: u64) {
        self.0 = (self.0 & !Self::S2AP) | (permission << Self::S2AP_OFFSET);
    }

    fn set_memory_attribute_write_back(&mut self) {
        self.0 = (self.0 & !Self::ATTR_INDEX) | Self::ATTR_WRITE_BACK;
    }

    /// Sets the Stage 2 output address attribute to Device-nGnRnE.
    /// (Used when passing through MMIO directly to the guest)
    fn set_memory_attribute_device(&mut self) {
        self.0 &= !Self::ATTR_INDEX;
    }
}

fn number_of_concatenated_page_tables(t0sz: u8, first_level: i8) -> usize {
    if t0sz > (43 - ((3 - first_level) as u8) * 9) {
        1
    } else {
        2usize.pow(((43 - ((3 - first_level) as u8) * 9) - t0sz) as u32)
    }
}

/// Allocates and zero-initializes a brand-new Stage 2 translation table for
/// one VCPU, and (re-)programs VTCR_EL2 to match it. VTCR_EL2 only encodes
/// hardware-derived translation-granule parameters (from ID_AA64MMFR0_EL1),
/// which are identical for every VCPU sharing a pCPU, so redundantly setting
/// it again here for a second/queued VCPU on the same pCPU is harmless.
///
/// Unlike the old `init_stage2_translation_table`, this does **not** touch
/// the live VTTBR_EL2: the caller must map this VCPU's guest memory into the
/// returned table (via `map_address_stage2`/`map_device_stage2`, passing
/// this table address) and only later make it the live translation regime
/// with `activate_stage2_translation_table`, once this VCPU actually starts
/// running (immediately for the first VCPU on a pCPU, or later from
/// `vm::try_yield_to_next_vcpu` for a queued one). This is what allows a
/// queued VCPU's table to be built up without corrupting the Stage 2
/// mappings of whichever VCPU is currently active on this pCPU.
/// Programs `VTCR_EL2` (Stage 2 translation granule/size parameters) for
/// the *current* physical CPU. This register is banked per pCPU by
/// hardware, so it must be (re-)set on every pCPU that will run a VCPU --
/// not just once when a Stage 2 table is first created (which only runs on
/// whichever pCPU happened to create that particular table). The computed
/// value only depends on hardware-derived translation-granule parameters
/// (from `ID_AA64MMFR0_EL1`), which are identical for every pCPU on this
/// system, so it is safe (and idempotent) to call this redundantly.
pub fn set_vtcr_el2_for_this_pcpu() {
    let ps = asm::get_id_aa64mmfr0_el1() & ID_AA64MMFR0_EL1_PARANGE;
    let (t0sz, initial_lookup_level) = match ps {
        0b000 => (32u64, 1i8),
        0b001 => (28u64, 1i8),
        0b010 => (24u64, 1i8),
        0b011 => (22u64, 1i8),
        0b100 => (20u64, 0i8),
        0b101 => (16u64, 0i8),
        _ => (16u64, 0i8),
    };
    let sl0 = if initial_lookup_level == 1 {
        0b01u64
    } else {
        0b10u64
    };
    let vtcr_el2: u64 = VTCR_EL2_RES1
        | (ps << VTCR_EL2_PS_BITS_OFFSET)
        | (0 << VTCR_EL2_TG0_BITS_OFFSET)
        | (0b11 << VTCR_EL2_SH0_BITS_OFFSET)
        | (0b11 << VTCR_EL2_ORGN0_BITS_OFFSET)
        | (0b11 << VTCR_EL2_IRGN0_BITS_OFFSET)
        | (sl0 << VTCR_EL2_SL0_BITS_OFFSET)
        | (t0sz << VTCR_EL2_T0SZ_BITS_OFFSET);

    unsafe {
        asm::set_vtcr_el2(vtcr_el2);
    }
}

pub fn create_stage2_translation_table() -> usize {
    let ps = asm::get_id_aa64mmfr0_el1() & ID_AA64MMFR0_EL1_PARANGE;
    let (t0sz, initial_lookup_level) = match ps {
        0b000 => (32u64, 1i8),
        0b001 => (28u64, 1i8),
        0b010 => (24u64, 1i8),
        0b011 => (22u64, 1i8),
        0b100 => (20u64, 0i8),
        0b101 => (16u64, 0i8),
        _ => (16u64, 0i8),
    };
    let number_of_tables = number_of_concatenated_page_tables(t0sz as u8, initial_lookup_level);
    let table = allocate_pages(number_of_tables, 12 + number_of_tables - 1).unwrap();
    for d in unsafe { from_raw_parts_mut(table as *mut Descriptor, number_of_tables * 512) } {
        d.init();
    }

    set_vtcr_el2_for_this_pcpu();

    table
}

/// Makes `table_address` (as returned by `create_stage2_translation_table`)
/// the live Stage 2 translation regime for this pCPU, tagged with `vmid` so
/// its TLB entries do not collide with those of other VCPUs previously (or
/// concurrently, on other pCPUs) scheduled with a different VMID. Must be
/// called before ever `eret`ing into (or resuming) the VCPU that owns this
/// table.
pub fn activate_stage2_translation_table(table_address: usize, vmid: u64) {
    let vttbr_el2 =
        ((vmid << VTTBR_VMID_BITS_OFFSET) & VTTBR_VMID) | (table_address as u64 & VTTBR_BADDR);
    unsafe { asm::set_vttbr_el2(vttbr_el2) };
}

fn _map_address_stage2(
    physical_address: &mut usize,
    intermediate_physical_address: &mut usize,
    remaining_size: &mut usize,
    table_address: usize,
    permission: u64,
    is_device: bool,
    level: i8,
    num_of_descriptors: usize,
) -> Result<(), ()> {
    let shift = 12 + 9 * (3 - level as usize);
    let index = (*intermediate_physical_address >> shift) & (num_of_descriptors - 1);
    let table = unsafe { from_raw_parts_mut(table_address as *mut Descriptor, num_of_descriptors) };

    if level == 3 {
        /* Page descriptor */
        for descriptor in table[index..num_of_descriptors].iter_mut() {
            descriptor.init();
            descriptor.set_output_address(*physical_address);
            descriptor.set_permission(permission);
            if is_device {
                descriptor.set_memory_attribute_device();
            } else {
                descriptor.set_memory_attribute_write_back();
            }
            descriptor.set_shareability(Shareability::InnerShareable);
            descriptor.validate_as_page_descriptor();
            *physical_address += PAGE_SIZE;
            *intermediate_physical_address += PAGE_SIZE;
            *remaining_size -= PAGE_SIZE;
            if *remaining_size == 0 {
                break;
            }
        }
        return Ok(());
    }

    for descriptor in table[index..num_of_descriptors].iter_mut() {
        let block_size = 1usize << shift;
        let mask = block_size - 1;
        if level >= 1
            && *remaining_size >= block_size
            && (*physical_address & mask) == 0
            && (*intermediate_physical_address & mask) == 0
        {
            /* Block descriptor */
            descriptor.init();
            descriptor.set_output_address(*physical_address);
            descriptor.set_permission(permission);
            if is_device {
                descriptor.set_memory_attribute_device();
            } else {
                descriptor.set_memory_attribute_write_back();
            }
            descriptor.set_shareability(Shareability::InnerShareable);
            descriptor.validate_as_block_descriptor();
            *physical_address += block_size;
            *intermediate_physical_address += block_size;
            *remaining_size -= block_size;
            if *remaining_size == 0 {
                return Ok(());
            }
            continue;
        }

        /* Table descriptor */
        let mut next_level_table_address = descriptor.get_next_level_table_address();
        if !descriptor.is_table_descriptor() {
            /* Create a translation table */
            next_level_table_address = allocate_pages(1, 12).map_err(|e| {
                println!("Failed to allocate new translation table: {:?}", e);
            })?;
            for d in unsafe { from_raw_parts_mut(next_level_table_address as *mut Descriptor, 512) }
            {
                d.init();
            }

            descriptor.init();
            descriptor.set_output_address(next_level_table_address);
            descriptor.validate_as_table_descriptor();
        }

        _map_address_stage2(
            physical_address,
            intermediate_physical_address,
            remaining_size,
            next_level_table_address,
            permission,
            is_device,
            level + 1,
            512,
        )?;
        if *remaining_size == 0 {
            break;
        }
    }
    Ok(())
}

/// Maps `map_size` bytes of normal (write-back) guest memory into the
/// Stage 2 translation table at `table_address` (as returned by
/// `create_stage2_translation_table`), which need not be the table
/// currently live in VTTBR_EL2.
pub fn map_address_stage2(
    table_address: usize,
    physical_address: usize,
    intermediate_physical_address: usize,
    map_size: usize,
    is_readable: bool,
    is_writable: bool,
) -> Result<(), ()> {
    map_address_stage2_internal(
        table_address,
        physical_address,
        intermediate_physical_address,
        map_size,
        is_readable,
        is_writable,
        false,
    )
}

/// Function for directly passthrough-mapping device memory, such as MMIO, to the guest.
/// (Used when passing the GICv2 Virtual CPU Interface to the guest). Like
/// `map_address_stage2`, operates on the explicit `table_address` rather
/// than whichever table is currently live in VTTBR_EL2.
pub fn map_device_stage2(
    table_address: usize,
    physical_address: usize,
    intermediate_physical_address: usize,
    map_size: usize,
    is_readable: bool,
    is_writable: bool,
) -> Result<(), ()> {
    map_address_stage2_internal(
        table_address,
        physical_address,
        intermediate_physical_address,
        map_size,
        is_readable,
        is_writable,
        true,
    )
}

fn map_address_stage2_internal(
    table_address: usize,
    mut physical_address: usize,
    mut intermediate_physical_address: usize,
    mut map_size: usize,
    is_readable: bool,
    is_writable: bool,
    is_device: bool,
) -> Result<(), ()> {
    if (map_size & ((1usize << PAGE_SHIFT) - 1)) != 0 {
        println!("Map size is not aligned.");
        return Err(());
    }
    let vtcr_el2 = asm::get_vtcr_el2();
    let sl0 = ((vtcr_el2 & VTCR_EL2_SL0) >> VTCR_EL2_SL0_BITS_OFFSET) as u8;
    let t0sz = ((vtcr_el2 & VTCR_EL2_T0SZ) >> VTCR_EL2_T0SZ_BITS_OFFSET) as u8;
    let initial_lookup_level: i8 = match sl0 {
        0b00 => 2,
        0b01 => 1,
        0b10 => 0,
        0b11 => 3,
        _ => unreachable!(),
    };
    let num_of_descriptors = number_of_concatenated_page_tables(t0sz, initial_lookup_level) * 512;

    _map_address_stage2(
        &mut physical_address,
        &mut intermediate_physical_address,
        &mut map_size,
        table_address,
        ((is_writable as u64) << 1) | (is_readable as u64),
        is_device,
        initial_lookup_level,
        num_of_descriptors,
    )?;

    /* Only flushing when `table_address` is the table currently live in
     * VTTBR_EL2 is correct (and sufficient): a table being built up for a
     * not-yet-activated VCPU (see `create_stage2_translation_table`) cannot
     * have any stale TLB entries yet, so there is nothing to invalidate for
     * it, and flushing here would only discard the *active* VCPU's TLB
     * entries for no reason. */
    if table_address == (asm::get_vttbr_el2() & VTTBR_BADDR) as usize {
        asm::flush_tlb_el1();
    }
    Ok(())
}
