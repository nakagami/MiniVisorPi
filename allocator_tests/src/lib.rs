pub mod paging {
    pub const PAGE_SHIFT: usize = 12;
}

#[path = "../../src/memory_allocator.rs"]
pub mod memory_allocator;

#[path = "../../src/vgic_lr.rs"]
pub mod vgic_lr;

#[path = "../../src/guest_memory.rs"]
pub mod guest_memory;
