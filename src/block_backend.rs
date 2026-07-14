//!
//! Runtime-selected physical block-storage backend
//!
//! Wraps whichever concrete storage driver was detected on the DTB (Virtio,
//! present on QEMU's `virt` machine, or SDHCI, present on e.g. physical
//! Raspberry Pi 4) behind a single [`BlockDevice`] implementation, so the
//! rest of the hypervisor (FAT32 layer, guest-facing Virtio-Blk MMIO device)
//! does not need to know which one is actually in use.
//!

use crate::drivers::block_device::BlockDevice;
use crate::drivers::{sdhci, virtio_blk};

pub enum BlockBackend {
    Invalid,
    Virtio(virtio_blk::VirtioBlk),
    Sdhci(sdhci::Sdhci),
}

impl BlockBackend {
    pub const fn invalid() -> Self {
        Self::Invalid
    }
}

impl BlockDevice for BlockBackend {
    fn read(&mut self, buffer_address: usize, block_address: u64, length: u64) -> Result<(), ()> {
        match self {
            Self::Invalid => Err(()),
            Self::Virtio(blk) => blk.read(buffer_address, block_address, length),
            Self::Sdhci(blk) => blk.read(buffer_address, block_address, length),
        }
    }

    fn write(&mut self, buffer_address: usize, block_address: u64, length: u64) -> Result<(), ()> {
        match self {
            Self::Invalid => Err(()),
            Self::Virtio(blk) => blk.write(buffer_address, block_address, length),
            Self::Sdhci(blk) => blk.write(buffer_address, block_address, length),
        }
    }
}
