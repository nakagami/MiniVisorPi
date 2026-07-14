//!
//! Common abstraction over physical block-storage backends
//!
//! Lets [`crate::fat32::Fat32`] read/write sectors without depending on a
//! single concrete driver type, so platforms without Virtio hardware (e.g.
//! physical Raspberry Pi 4) can supply a different backend, such as
//! [`crate::drivers::sdhci::Sdhci`], through the same code path.
//!

pub trait BlockDevice {
    fn read(&mut self, buffer_address: usize, block_address: u64, length: u64) -> Result<(), ()>;
    fn write(&mut self, buffer_address: usize, block_address: u64, length: u64) -> Result<(), ()>;
}
