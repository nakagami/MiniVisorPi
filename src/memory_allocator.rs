//!
//! Memory Allocator
//!

use crate::paging::PAGE_SHIFT;

use core::cell::UnsafeCell;

pub struct MemoryAllocator {
    free_size: usize,
    first_entry: Option<u32>,
    free_list: [Option<u32>; Self::NUM_OF_FREE_LIST],
    memory_entry_pool: MemoryEntryPool,
}

type MemoryEntryPool = UnsafeCell<[MemoryEntry; MemoryAllocator::POOL_SIZE]>;

#[derive(Clone, Copy)]
struct MemoryEntry {
    previous: Option<u32>,
    next: Option<u32>,
    list_prev: Option<u32>,
    list_next: Option<u32>,
    start: usize,
    end: usize,
    enabled: bool,
    id: u32,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum MemoryError {
    NoEntry,
    InvalidEntry,
    NoMemory,
    InvalidRequest,
}

impl MemoryAllocator {
    const POOL_SIZE: usize = 64;
    const NUM_OF_FREE_LIST: usize = 12;

    pub const fn new() -> Self {
        Self {
            free_size: 0,
            free_list: [None; Self::NUM_OF_FREE_LIST],
            memory_entry_pool: UnsafeCell::new([MemoryEntry::new(); Self::POOL_SIZE]),
            first_entry: None,
        }
    }

    fn create_memory_entry(
        pool: &MemoryEntryPool,
    ) -> Result<&'static mut MemoryEntry, MemoryError> {
        for (i, e) in unsafe { &mut *pool.get() }.iter_mut().enumerate() {
            if !e.enabled {
                *e = MemoryEntry::new();
                e.id = i as u32;
                e.enabled = true;
                return Ok(e);
            }
        }
        Err(MemoryError::NoEntry)
    }

    fn get_memory_entry(
        pool: &MemoryEntryPool,
        index: u32,
    ) -> Result<&'static mut MemoryEntry, MemoryError> {
        let index = index as usize;
        if index >= Self::POOL_SIZE {
            Err(MemoryError::InvalidEntry)
        } else {
            Ok(unsafe { &mut (*pool.get())[index] })
        }
    }

    fn search_entry_containing_address_mut(
        pool: &MemoryEntryPool,
        first_entry: Option<u32>,
        address: usize,
    ) -> Option<&mut MemoryEntry> {
        let first_entry = first_entry?;
        let mut entry = Self::get_memory_entry(pool, first_entry).ok()?;
        while entry.get_start_address() < address && entry.get_end_address() < address {
            if let Some(t) = Self::get_next_entry(pool, entry) {
                entry = t;
            } else {
                return None;
            }
        }
        if address >= entry.get_start_address() && address <= entry.get_end_address() {
            Some(entry)
        } else {
            None
        }
    }

    fn search_entry_previous_address_mut(
        pool: &MemoryEntryPool,
        first_entry: Option<u32>,
        address: usize,
    ) -> Option<&mut MemoryEntry> {
        let first_entry = first_entry?;
        let mut entry = Self::get_memory_entry(pool, first_entry).ok()?;
        while entry.get_start_address() < address {
            if let Some(t) = Self::get_next_entry(pool, entry) {
                entry = t;
            } else {
                return if entry.get_end_address() <= address {
                    Some(entry)
                } else {
                    Self::get_prev_entry(pool, entry)
                };
            }
        }
        Self::get_prev_entry(pool, entry)
    }

    fn define_used_memory(
        &mut self,
        start: usize,
        size: usize,
        align_order: usize,
        target_entry: &mut Option<&mut MemoryEntry>,
    ) -> Result<(), MemoryError> {
        if size == 0 {
            return Err(MemoryError::InvalidRequest);
        } else if self.free_size < size {
            return Err(MemoryError::NoMemory);
        }
        if align_order != 0 {
            let (aligned_start, aligned_size) =
                Self::align_address_and_size(start, size, align_order);
            return self.define_used_memory(aligned_start, aligned_size, 0, target_entry);
        }
        let entry = if let Some(t) = target_entry {
            t
        } else if let Some(t) = Self::search_entry_containing_address_mut(
            &self.memory_entry_pool,
            self.first_entry,
            start,
        ) {
            t
        } else {
            return Err(MemoryError::InvalidRequest);
        };
        /* `start`/`size` may legitimately span past the end of the single
         * free entry containing `start`: two independently-computed "used"
         * regions (e.g. this hypervisor's own conservatively page-rounded
         * stack reservation and a DTB relocated by the bootloader) can end
         * up overlapping on some boot paths/media, even though neither
         * computation is wrong on its own. Whatever lies beyond this free
         * entry's end is -- by definition of the free-list model used here
         * -- already excluded from the free pool, so only the portion that
         * is actually still free needs to be carved out here; blindly using
         * the full requested `size` in that case would make the branches
         * below compute a new entry with start > end and hit the
         * `start <= end` assertion in `MemoryEntry::set_range`. */
        let size = if entry.get_end_address() < get_end_address(start, size) {
            entry.get_end_address() - start + 1
        } else {
            size
        };
        if entry.get_start_address() == start {
            if entry.get_end_address() == get_end_address(start, size) {
                /* Delete the entry */
                if entry.is_first_entry() {
                    if let Some(next) = Self::get_next_entry(&self.memory_entry_pool, entry) {
                        self.first_entry = Some(next.id);
                    } else {
                        self.first_entry = None;
                    }
                }
                Self::unchain_entry_from_free_list(
                    &self.memory_entry_pool,
                    &mut self.free_list,
                    entry,
                );
                entry.delete(&self.memory_entry_pool);
                if target_entry.is_some() {
                    *target_entry = None;
                }
            } else {
                let old_size = entry.get_size();
                entry.set_range(start + size, entry.get_end_address());
                Self::chain_entry_to_free_list(
                    &self.memory_entry_pool,
                    &mut self.free_list,
                    entry,
                    Some(old_size),
                );
            }
        } else if entry.get_end_address() == start {
            if size != 1 {
                return Err(MemoryError::InvalidRequest);
            }
            /* Allocate 1 byte of end_address */
            entry.set_range(entry.get_start_address(), start - 1);
            Self::chain_entry_to_free_list(
                &self.memory_entry_pool,
                &mut self.free_list,
                entry,
                Some(entry.get_size() + 1),
            );
        } else if entry.get_end_address() == get_end_address(start, size) {
            let old_size = entry.get_size();
            entry.set_range(entry.get_start_address(), start - 1);
            Self::chain_entry_to_free_list(
                &self.memory_entry_pool,
                &mut self.free_list,
                entry,
                Some(old_size),
            );
        } else {
            let new_entry = Self::create_memory_entry(&self.memory_entry_pool)?;
            let old_size = entry.get_size();
            new_entry.set_range(start + size, entry.get_end_address());
            entry.set_range(entry.get_start_address(), start - 1);
            if let Some(next) = Self::get_next_entry(&self.memory_entry_pool, entry) {
                new_entry.chain_after_me(next);
            }
            entry.chain_after_me(new_entry);
            Self::chain_entry_to_free_list(
                &self.memory_entry_pool,
                &mut self.free_list,
                entry,
                Some(old_size),
            );
            Self::chain_entry_to_free_list(
                &self.memory_entry_pool,
                &mut self.free_list,
                new_entry,
                None,
            );
        }
        self.free_size -= size;
        Ok(())
    }

    fn define_free_memory(&mut self, start: usize, size: usize) -> Result<(), MemoryError> {
        if size == 0 {
            return Err(MemoryError::InvalidRequest);
        }
        let entry = if let Some(e) = Self::search_entry_previous_address_mut(
            &self.memory_entry_pool,
            self.first_entry,
            start,
        ) {
            e
        } else if let Ok(e) =
            Self::get_memory_entry(&self.memory_entry_pool, self.first_entry.unwrap())
        {
            e
        } else {
            return Err(MemoryError::InvalidEntry);
        };
        let end = get_end_address(start, size);

        if entry.get_start_address() <= start && entry.get_end_address() >= end {
            /* already freed */
            return Err(MemoryError::InvalidRequest);
        } else if entry.get_end_address() >= start && !entry.is_first_entry() {
            /* Free duplicated area */
            return self.define_free_memory(
                entry.get_end_address() + 1,
                end - entry.get_end_address() + 2,
            );
        } else if entry.get_end_address() == end {
            /* Free duplicated area */
            /* entry may be first entry */
            return self.define_free_memory(start, size - entry.get_size());
        }

        let mut processed = false;
        let old_size = entry.get_size();
        let address_after_entry = entry.get_end_address() + 1;

        if address_after_entry == start {
            entry.set_range(entry.get_start_address(), end);
            processed = true;
        }

        if entry.is_first_entry() && entry.get_start_address() == end + 1 {
            entry.set_range(start, entry.get_end_address());
            processed = true;
        }

        if let Some(next) = Self::get_next_entry(&self.memory_entry_pool, entry) {
            if next.get_start_address() <= start {
                assert!(!processed);
                return if next.get_end_address() >= end {
                    Err(MemoryError::InvalidRequest) /* already freed */
                } else {
                    self.define_free_memory(
                        next.get_end_address() + 1,
                        end - next.get_end_address(),
                    )
                };
            }
            if next.get_start_address() == end + 1 {
                let next_old_size = next.get_size();
                next.set_range(start, next.get_end_address());
                Self::chain_entry_to_free_list(
                    &self.memory_entry_pool,
                    &mut self.free_list,
                    next,
                    Some(next_old_size),
                );
                processed = true;
            }

            if (next.get_start_address() == entry.get_end_address() + 1)
                || (processed && address_after_entry >= next.get_start_address())
            {
                entry.set_range(
                    entry.get_start_address(),
                    entry.get_end_address().max(next.get_end_address()),
                );

                Self::unchain_entry_from_free_list(
                    &self.memory_entry_pool,
                    &mut self.free_list,
                    next,
                );
                next.delete(&self.memory_entry_pool);
            }
            if processed {
                self.free_size += size;
                Self::chain_entry_to_free_list(
                    &self.memory_entry_pool,
                    &mut self.free_list,
                    entry,
                    Some(old_size),
                );
                return Ok(());
            }
            let new_entry = Self::create_memory_entry(&self.memory_entry_pool)?;
            new_entry.set_range(start, end);
            if new_entry.get_end_address() < entry.get_start_address() {
                if let Some(prev_entry) = Self::get_prev_entry(&self.memory_entry_pool, entry) {
                    assert!(prev_entry.get_end_address() < new_entry.get_start_address());
                    prev_entry.chain_after_me(new_entry);
                    new_entry.chain_after_me(entry);
                } else {
                    self.first_entry = Some(new_entry.id);
                    new_entry.chain_after_me(entry);
                }
            } else {
                next.set_prev_entry(new_entry);
                new_entry.set_next_entry(next);
                entry.chain_after_me(new_entry);
            }
            self.free_size += size;
            Self::chain_entry_to_free_list(
                &self.memory_entry_pool,
                &mut self.free_list,
                entry,
                Some(old_size),
            );
            Self::chain_entry_to_free_list(
                &self.memory_entry_pool,
                &mut self.free_list,
                new_entry,
                None,
            );
            Ok(())
        } else {
            if processed {
                self.free_size += size;
                Self::chain_entry_to_free_list(
                    &self.memory_entry_pool,
                    &mut self.free_list,
                    entry,
                    Some(old_size),
                );
                return Ok(());
            }
            let new_entry = Self::create_memory_entry(&self.memory_entry_pool)?;
            new_entry.set_range(start, end);
            if entry.get_end_address() < new_entry.get_start_address() {
                entry.chain_after_me(new_entry);
            } else {
                if let Some(prev_entry) = Self::get_prev_entry(&self.memory_entry_pool, entry) {
                    assert!(prev_entry.get_end_address() < entry.get_start_address());
                    prev_entry.chain_after_me(new_entry);
                } else {
                    self.first_entry = Some(new_entry.id);
                }
                new_entry.chain_after_me(entry);
            }
            self.free_size += size;
            Self::chain_entry_to_free_list(
                &self.memory_entry_pool,
                &mut self.free_list,
                entry,
                Some(old_size),
            );
            Self::chain_entry_to_free_list(
                &self.memory_entry_pool,
                &mut self.free_list,
                new_entry,
                None,
            );
            Ok(())
        }
    }

    pub fn allocate(&mut self, size: usize, align_order: usize) -> Result<usize, MemoryError> {
        self.allocate_with_address_limit(size, align_order, usize::MAX)
    }

    /// Like [`allocate`](Self::allocate), but the returned region is
    /// guaranteed to lie entirely below `address_limit` (an exclusive
    /// upper bound). Used for buffers handed to DMA engines with a
    /// limited address reach: the BCM2711 PCIe inbound window (RC_BAR2,
    /// see `drivers/pcie_brcm.rs`) only maps RAM below 4 GiB for the
    /// VL805 xHCI, and SDHCI/GENET share the limitation -- even on boards
    /// (e.g. the 8 GiB Pi4) where more RAM exists above that boundary.
    pub fn allocate_below(
        &mut self,
        size: usize,
        align_order: usize,
        address_limit: usize,
    ) -> Result<usize, MemoryError> {
        self.allocate_with_address_limit(size, align_order, address_limit)
    }

    fn allocate_with_address_limit(
        &mut self,
        size: usize,
        align_order: usize,
        address_limit: usize,
    ) -> Result<usize, MemoryError> {
        if size == 0 {
            return Err(MemoryError::InvalidRequest);
        } else if self.free_size < size {
            return Err(MemoryError::NoMemory);
        }
        let page_order = Self::size_to_page_order(size);
        for i in page_order..Self::NUM_OF_FREE_LIST {
            let first_entry = if let Some(t) = self.free_list[i] {
                Self::get_memory_entry(&self.memory_entry_pool, t)?
            } else {
                continue;
            };

            let mut entry = Some(first_entry);
            while let Some(e) = entry {
                /* Entries use an *inclusive* end address; clamp it to the
                 * limit to get the portion of this entry the allocation
                 * is allowed to use. */
                let start = e.get_start_address();
                let end = e
                    .get_end_address()
                    .min(address_limit.saturating_sub(1));
                let available_size = if end >= start { end - start + 1 } else { 0 };
                if available_size >= size {
                    let address_to_allocate = if align_order != 0 {
                        let (aligned_address, aligned_available_size) =
                            Self::align_address_and_available_size(
                                start,
                                available_size,
                                align_order,
                            );
                        if aligned_available_size < size {
                            entry = e.list_next.and_then(|n| {
                                Self::get_memory_entry(&self.memory_entry_pool, n).ok()
                            });
                            continue;
                        }
                        aligned_address
                    } else {
                        e.get_start_address()
                    };
                    self.define_used_memory(address_to_allocate, size, 0, &mut Some(e))?;
                    return Ok(address_to_allocate);
                }
                entry = e
                    .list_next
                    .and_then(|n| Self::get_memory_entry(&self.memory_entry_pool, n).ok());
            }
        }
        Err(MemoryError::NoMemory)
    }

    pub fn reserve_memory(
        &mut self,
        start_address: usize,
        size: usize,
        align_order: usize,
    ) -> Result<(), MemoryError> {
        self.define_used_memory(start_address, size, align_order, &mut None)
    }

    pub fn free(&mut self, start: usize, size: usize) -> Result<(), MemoryError> {
        if self.free_size == 0 {
            let first_entry = Self::create_memory_entry(&self.memory_entry_pool)?;
            first_entry.set_range(start, get_end_address(start, size));
            Self::chain_entry_to_free_list(
                &self.memory_entry_pool,
                &mut self.free_list,
                first_entry,
                None,
            );
            self.first_entry = Some(first_entry.id);
            self.free_size = size;
        } else {
            self.define_free_memory(start, size)?;
        }
        Ok(())
    }

    fn unchain_entry_from_free_list(
        pool: &MemoryEntryPool,
        free_list: &mut [Option<u32>; Self::NUM_OF_FREE_LIST],
        entry: &mut MemoryEntry,
    ) {
        let order = Self::size_to_page_order(entry.get_size());
        if free_list[order] == Some(entry.id) {
            free_list[order] = entry.list_next;
        }
        entry.unchain_from_freelist(pool);
    }

    fn chain_entry_to_free_list(
        pool: &MemoryEntryPool,
        free_list: &mut [Option<u32>; Self::NUM_OF_FREE_LIST],
        entry: &mut MemoryEntry,
        old_size: Option<usize>,
    ) {
        let new_order = Self::size_to_page_order(entry.get_size());
        if let Some(old_size) = old_size {
            if old_size == entry.get_size() {
                return;
            }
            let old_order = Self::size_to_page_order(old_size);
            if free_list[old_order] == Some(entry.id) {
                free_list[old_order] = entry.list_next;
            }
            entry.unchain_from_freelist(pool);
        }
        assert_eq!(entry.list_next, None);
        assert_eq!(entry.list_prev, None);

        if let Some(mut list_entry) =
            free_list[new_order].and_then(|i| Self::get_memory_entry(pool, i).ok())
        {
            if list_entry.get_size() >= entry.get_size() {
                list_entry.list_prev = Some(entry.id);
                entry.list_next = Some(list_entry.id);
                free_list[new_order] = Some(entry.id);
            } else {
                loop {
                    if let Some(next_entry) = list_entry
                        .list_next
                        .and_then(|n| Self::get_memory_entry(pool, n).ok())
                    {
                        if next_entry.get_size() >= entry.get_size() {
                            list_entry.list_next = Some(entry.id);
                            entry.list_prev = Some(list_entry.id);
                            entry.list_next = Some(next_entry.id);
                            next_entry.list_prev = Some(entry.id);
                            break;
                        }
                        list_entry = next_entry;
                    } else {
                        list_entry.list_next = Some(entry.id);
                        entry.list_prev = Some(list_entry.id);
                        break;
                    }
                }
            }
        } else {
            free_list[new_order] = Some(entry.id);
        }
    }

    fn get_next_entry(
        pool: &MemoryEntryPool,
        entry: &MemoryEntry,
    ) -> Option<&'static mut MemoryEntry> {
        entry
            .next
            .and_then(|n| Self::get_memory_entry(pool, n).ok())
    }

    fn get_prev_entry(
        pool: &MemoryEntryPool,
        entry: &MemoryEntry,
    ) -> Option<&'static mut MemoryEntry> {
        entry
            .previous
            .and_then(|n| Self::get_memory_entry(pool, n).ok())
    }

    #[inline]
    const fn size_to_page_order(size: usize) -> usize {
        let mut order = 0;
        while size > (1 << (order + PAGE_SHIFT)) {
            order += 1;
            if order == Self::NUM_OF_FREE_LIST - 1 {
                return order;
            }
        }
        order
    }

    #[inline]
    const fn align_address_and_size(
        address: usize,
        size: usize,
        align_order: usize,
    ) -> (usize /* address */, usize /* size */) {
        let align_size = 1 << align_order;
        let mask = !(align_size - 1);
        let aligned_address = address & mask;
        (
            aligned_address,
            ((size + (address - aligned_address) - 1) & mask) + align_size,
        )
    }

    #[inline]
    const fn align_address_and_available_size(
        start: usize,
        size: usize,
        align_order: usize,
    ) -> (usize, usize) {
        if start == 0 {
            (0, size)
        } else {
            let align_size = 1 << align_order;
            let mask = !(align_size - 1);
            let aligned_address = ((start - 1) & mask) + align_size;
            if size > (aligned_address - start) {
                (aligned_address, size - (aligned_address - start))
            } else {
                (aligned_address, 0)
            }
        }
    }
}

impl MemoryEntry {
    const fn new() -> Self {
        Self {
            previous: None,
            next: None,
            list_prev: None,
            list_next: None,
            start: 0,
            end: 0,
            enabled: false,
            id: 0,
        }
    }

    pub fn delete(&mut self, pool: &MemoryEntryPool) {
        if let Some(previous) = MemoryAllocator::get_prev_entry(pool, self) {
            if let Some(next) = MemoryAllocator::get_next_entry(pool, self) {
                previous.chain_after_me(next);
            } else {
                previous.unset_next_entry();
            }
        } else if let Some(next) = MemoryAllocator::get_next_entry(pool, self) {
            next.unset_prev_entry();
        }
        self.previous = None;
        self.next = None;
        self.enabled = false;
    }

    pub fn set_range(&mut self, start: usize, end: usize) {
        /* `end` is inclusive (get_size() == end - start + 1), so a valid
         * single-byte entry legitimately has start == end. Rejecting that
         * with `start < end` spuriously panics whenever the allocator needs
         * to shrink an entry down to exactly one remaining byte. */
        assert!(start <= end);
        self.start = start;
        self.end = end;
    }

    pub fn get_start_address(&self) -> usize {
        self.start
    }

    pub fn get_end_address(&self) -> usize {
        self.end
    }

    pub fn set_prev_entry(&mut self, prev: &mut Self) {
        self.previous = Some(prev.id);
    }

    pub fn unset_prev_entry(&mut self) {
        self.previous = None;
    }

    pub fn set_next_entry(&mut self, next: &mut Self) {
        self.next = Some(next.id);
    }

    pub fn unset_next_entry(&mut self) {
        self.next = None;
    }

    pub fn get_size(&self) -> usize {
        self.end - self.start + 1
    }

    pub fn chain_after_me(&mut self, entry: &mut Self) {
        self.next = Some(entry.id);
        entry.previous = Some(self.id);
    }

    pub fn is_first_entry(&self) -> bool {
        self.previous.is_none()
    }

    pub fn unchain_from_freelist(&mut self, pool: &MemoryEntryPool) {
        if let Some(prev_entry) = self
            .list_prev
            .and_then(|i| MemoryAllocator::get_memory_entry(pool, i).ok())
        {
            prev_entry.list_next = self.list_next;
        }
        if let Some(next_entry) = self
            .list_next
            .and_then(|i| MemoryAllocator::get_memory_entry(pool, i).ok())
        {
            next_entry.list_prev = self.list_prev;
        }
        self.list_next = None;
        self.list_prev = None;
    }
}

const fn get_end_address(address: usize, size: usize) -> usize {
    address + size - 1
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOTAL_SIZE: usize = 0x20_0000;

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    struct Range {
        start: usize,
        size: usize,
    }

    impl Range {
        fn end(self) -> usize {
            self.start + self.size - 1
        }
    }

    #[derive(Clone, Copy)]
    struct Rng(u64);

    impl Rng {
        fn new(seed: u64) -> Self {
            Self(seed)
        }

        fn next_u64(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }

        fn next_usize(&mut self, upper: usize) -> usize {
            if upper <= 1 {
                0
            } else {
                (self.next_u64() as usize) % upper
            }
        }
    }

    fn overlap(a: Range, b: Range) -> bool {
        a.start <= b.end() && b.start <= a.end()
    }

    fn assert_non_overlapping(ranges: &[Range]) {
        for (i, a) in ranges.iter().enumerate() {
            for b in &ranges[i + 1..] {
                assert!(!overlap(*a, *b), "overlap detected: {a:?} vs {b:?}");
            }
        }
    }

    fn enabled_entries(allocator: &MemoryAllocator) -> Vec<Range> {
        let pool = unsafe { &*allocator.memory_entry_pool.get() };
        pool.iter()
            .filter(|entry| entry.enabled)
            .map(|entry| Range {
                start: entry.start,
                size: entry.get_size(),
            })
            .collect()
    }

    fn sorted_entries(allocator: &MemoryAllocator) -> Vec<Range> {
        let mut entries = Vec::new();
        let mut current = allocator.first_entry;
        while let Some(id) = current {
            let entry = MemoryAllocator::get_memory_entry(&allocator.memory_entry_pool, id)
                .expect("valid entry id");
            assert!(entry.enabled, "disabled entry in main chain: {id}");
            entries.push(Range {
                start: entry.start,
                size: entry.get_size(),
            });
            current = entry.next;
        }
        entries
    }

    fn assert_allocator_invariants(allocator: &MemoryAllocator, expected_free: &[Range]) {
        let mut chain_entries = sorted_entries(allocator);
        let mut pool_entries = enabled_entries(allocator);
        chain_entries.sort_by_key(|range| range.start);
        pool_entries.sort_by_key(|range| range.start);
        assert_eq!(chain_entries, pool_entries, "pool and chain disagree");
        assert_non_overlapping(&chain_entries);

        for (prev, next) in chain_entries.iter().zip(chain_entries.iter().skip(1)) {
            assert!(
                prev.end() + 1 < next.start,
                "adjacent/overlapping free entries were not coalesced: {prev:?} {next:?}"
            );
        }

        let pool = unsafe { &*allocator.memory_entry_pool.get() };
        let mut seen_free_list_ids = Vec::new();
        for (order, head) in allocator.free_list.iter().copied().enumerate() {
            let mut current = head;
            let mut last_size = 0;
            while let Some(id) = current {
                let entry = &pool[id as usize];
                assert!(entry.enabled, "disabled entry in free list {order}: {id}");
                let entry_order = MemoryAllocator::size_to_page_order(entry.get_size());
                assert_eq!(entry_order, order, "entry {id} is in wrong free list order");
                assert!(
                    last_size <= entry.get_size(),
                    "free list {order} is not sorted by size"
                );
                last_size = entry.get_size();
                seen_free_list_ids.push(id);
                current = entry.list_next;
            }
        }

        seen_free_list_ids.sort_unstable();
        seen_free_list_ids.dedup();
        assert_eq!(
            seen_free_list_ids.len(),
            chain_entries.len(),
            "free lists do not cover every enabled entry exactly once"
        );

        let expected_total: usize = expected_free.iter().map(|range| range.size).sum();
        assert_eq!(
            allocator.free_size, expected_total,
            "free_size accounting mismatch"
        );

        let mut actual_free = chain_entries;
        actual_free.sort_by_key(|range| range.start);
        let mut expected = expected_free.to_vec();
        expected.sort_by_key(|range| range.start);
        assert_eq!(actual_free, expected, "allocator free ranges diverged");
    }

    fn insert_free_range(free_ranges: &mut Vec<Range>, range: Range) {
        free_ranges.push(range);
        free_ranges.sort_by_key(|free| free.start);

        let mut merged: Vec<Range> = Vec::with_capacity(free_ranges.len());
        for free in free_ranges.drain(..) {
            if let Some(last) = merged.last_mut() {
                if last.end() + 1 >= free.start {
                    let new_end = last.end().max(free.end());
                    last.size = new_end - last.start + 1;
                    continue;
                }
            }
            merged.push(free);
        }
        *free_ranges = merged;
    }

    fn remove_free_subrange(free_ranges: &mut Vec<Range>, target: Range) {
        let index = free_ranges
            .iter()
            .position(|free| target.start >= free.start && target.end() <= free.end())
            .expect("target must be free");
        let original = free_ranges.remove(index);
        if original.start < target.start {
            free_ranges.push(Range {
                start: original.start,
                size: target.start - original.start,
            });
        }
        if target.end() < original.end() {
            free_ranges.push(Range {
                start: target.end() + 1,
                size: original.end() - target.end(),
            });
        }
        free_ranges.sort_by_key(|range| range.start);
    }

    fn reserve_some_ranges(
        allocator: &mut MemoryAllocator,
        free_ranges: &mut Vec<Range>,
        reserved: &mut Vec<Range>,
    ) {
        for range in [
            Range {
                start: 0x1000,
                size: 0x7000,
            },
            Range {
                start: 0x20000,
                size: 0x18000,
            },
            Range {
                start: 0x180000,
                size: 0x30000,
            },
        ] {
            allocator
                .reserve_memory(range.start, range.size, 0)
                .expect("reserve_memory should succeed");
            remove_free_subrange(free_ranges, range);
            reserved.push(range);
            assert_allocator_invariants(allocator, free_ranges);
        }
    }

    #[test]
    fn free_size_tracks_exact_frees() {
        let mut allocator = MemoryAllocator::new();
        allocator.free(0, 1024).unwrap();
        assert_eq!(allocator.free_size, 1024);

        let first = allocator.allocate(128, 0).unwrap();
        assert_eq!(first, 0);
        assert_eq!(allocator.free_size, 896);

        allocator.free(first, 128).unwrap();
        assert_eq!(
            allocator.free_size, 1024,
            "free() should restore exactly the bytes that were allocated"
        );
    }

    #[test]
    fn allocate_can_consume_last_free_entry() {
        let mut allocator = MemoryAllocator::new();
        allocator.free(0x2000, 256).unwrap();

        let address = allocator.allocate(256, 0).unwrap();
        assert_eq!(address, 0x2000);
        assert_eq!(allocator.first_entry, None);
        assert_eq!(allocator.free_size, 0);
    }

    /// Regression test for a real Raspberry Pi 4 microSD-boot panic: the
    /// bootloader can place the relocated DTB and this hypervisor's own
    /// (conservatively page-rounded) entry stack such that the two
    /// independently-computed "used" regions overlap, even though neither
    /// computation is wrong on its own (see `setup_memory()` in main.rs).
    /// Reserving the stack region used to hit the `start <= end` assertion
    /// in `MemoryEntry::set_range` whenever the requested range extended
    /// past the end of the free entry that contains its start address.
    #[test]
    fn reserve_memory_tolerates_overlap_with_prior_reservation() {
        let mut allocator = MemoryAllocator::new();
        allocator.free(0, TOTAL_SIZE).unwrap();

        /* Reserve a "DTB"-like region first. */
        let dtb_start = 0x1000;
        let dtb_size = 0x1000;
        allocator.reserve_memory(dtb_start, dtb_size, 0).unwrap();

        /* Reserve a "stack"-like region that starts before the DTB but
         * extends past its start, overlapping it -- this must not panic,
         * and must not double-count the already-reserved overlap. */
        let stack_start = dtb_start - 0x800;
        let stack_size = 0x1000;
        allocator
            .reserve_memory(stack_start, stack_size, 0)
            .expect("overlapping reservation should be tolerated, not panic");

        let expected_free = vec![
            Range {
                start: 0,
                size: stack_start,
            },
            Range {
                start: dtb_start + dtb_size,
                size: TOTAL_SIZE - (dtb_start + dtb_size),
            },
        ];
        assert_allocator_invariants(&allocator, &expected_free);
    }

    #[test]
    fn fuzz_allocate_free_and_reserve() {
        for seed in 0..64u64 {
            let mut allocator = MemoryAllocator::new();
            allocator.free(0, TOTAL_SIZE).unwrap();

            let mut rng = Rng::new(0xC0FFEE_u64 ^ seed);
            let mut free_ranges = vec![Range {
                start: 0,
                size: TOTAL_SIZE,
            }];
            let mut reserved = Vec::new();
            let mut allocations = Vec::new();

            reserve_some_ranges(&mut allocator, &mut free_ranges, &mut reserved);

            for _ in 0..2000 {
                let fragment_pressure = enabled_entries(&allocator).len();
                let do_alloc = allocations.is_empty()
                    || (fragment_pressure < MemoryAllocator::POOL_SIZE - 8
                        && rng.next_usize(100) < 65);
                if do_alloc {
                    let size = 0x100 + rng.next_usize(0x7F00);
                    let align_order = match rng.next_usize(8) {
                        0 => 0,
                        1 => 1,
                        2 => 2,
                        3 => 3,
                        4 => 4,
                        5 => 8,
                        6 => 12,
                        _ => 16,
                    };
                    let address = match allocator.allocate(size, align_order) {
                        Ok(address) => address,
                        Err(MemoryError::NoMemory) => {
                            assert_allocator_invariants(&allocator, &free_ranges);
                            continue;
                        }
                        Err(MemoryError::NoEntry) => {
                            assert_eq!(
                                enabled_entries(&allocator).len(),
                                MemoryAllocator::POOL_SIZE
                            );
                            assert_allocator_invariants(&allocator, &free_ranges);
                            continue;
                        }
                        Err(err) => panic!("unexpected allocation error: {err:?} (seed {seed})"),
                    };
                    let range = Range {
                        start: address,
                        size,
                    };
                    assert_non_overlapping(&allocations);
                    for live in allocations.iter().chain(reserved.iter()) {
                        assert!(
                            !overlap(*live, range),
                            "allocator returned overlapping range {range:?} with live {live:?} (seed {seed})"
                        );
                    }
                    remove_free_subrange(&mut free_ranges, range);
                    allocations.push(range);
                } else {
                    let index = rng.next_usize(allocations.len());
                    let range = allocations.swap_remove(index);
                    allocator.free(range.start, range.size).unwrap();
                    insert_free_range(&mut free_ranges, range);
                }

                assert_non_overlapping(&allocations);
                assert_allocator_invariants(&allocator, &free_ranges);
            }
        }
    }
}
