//!
//! USB Mass Storage (Bulk-Only Transport / SCSI) driver
//!
//! Implements the minimal BOT + SCSI subset needed to expose a directly attached
//! USB flash drive as this hypervisor's [`crate::drivers::block_device::BlockDevice`]
//! fallback backend. Transport is always provided by the shared xHCI host
//! controller driver, so the same code path serves USB 2.0 (Low/Full/High
//! speed) and USB 3.0 (SuperSpeed) mass-storage devices; the xHCI layer hides
//! the speed-specific EP0 sizing and SuperSpeed endpoint-companion/Max-Burst
//! details from this BOT/SCSI layer. This is a from-scratch bring-up driver
//! that has **not** been validated on physical hardware yet: command
//! sequencing follows U-Boot's USB mass-storage stack, but real-hardware
//! testing is still required before relying on it.
//!

use crate::drivers::block_device::BlockDevice;
use crate::drivers::xhci::XhciMassStorageDevice;

const CBW_SIGNATURE: u32 = 0x4342_5355;
const CSW_SIGNATURE: u32 = 0x5342_5355;
const BOT_FLAG_IN: u8 = 0x80;
const BOT_LUN: u8 = 0;

const SCSI_TEST_UNIT_READY: u8 = 0x00;
const SCSI_REQUEST_SENSE: u8 = 0x03;
const SCSI_INQUIRY: u8 = 0x12;
const SCSI_READ_CAPACITY_10: u8 = 0x25;
const SCSI_READ_10: u8 = 0x28;
const SCSI_WRITE_10: u8 = 0x2A;
const SCSI_SERVICE_ACTION_IN_16: u8 = 0x9E;
const SCSI_SAI_READ_CAPACITY_16: u8 = 0x10;

pub struct UsbMassStorage {
    device: XhciMassStorageDevice,
    tag: u32,
    block_size: u32,
    last_lba: u64,
}

impl UsbMassStorage {
    pub fn new(mut device: XhciMassStorageDevice) -> Result<Self, ()> {
        let mut inquiry = [0u8; 36];
        let _ = Self::execute_scsi_command_inner(
            &mut device,
            1,
            &[SCSI_INQUIRY, 0, 0, 0, inquiry.len() as u8, 0],
            inquiry.as_mut_ptr() as usize,
            inquiry.len(),
            true,
        )?;

        for _ in 0..4 {
            if Self::test_unit_ready_inner(&mut device, 2).is_ok() {
                break;
            }
            let _ = Self::request_sense_inner(&mut device, 3);
        }

        let (last_lba, block_size) = Self::read_capacity_inner(&mut device, 4)?;
        if block_size == 0 {
            println!("USB BOT: invalid logical block size 0");
            return Err(());
        }
        Ok(Self {
            device,
            tag: 5,
            block_size,
            last_lba,
        })
    }

    fn test_unit_ready_inner(device: &mut XhciMassStorageDevice, tag: u32) -> Result<(), ()> {
        Self::execute_scsi_command_inner(device, tag, &[SCSI_TEST_UNIT_READY, 0, 0, 0, 0, 0], 0, 0, false)
            .map(|_| ())
    }

    fn request_sense_inner(device: &mut XhciMassStorageDevice, tag: u32) -> Result<(), ()> {
        let mut sense = [0u8; 18];
        let _ = Self::execute_scsi_command_inner(
            device,
            tag,
            &[SCSI_REQUEST_SENSE, 0, 0, 0, sense.len() as u8, 0],
            sense.as_mut_ptr() as usize,
            sense.len(),
            true,
        )?;
        Ok(())
    }

    fn read_capacity_inner(device: &mut XhciMassStorageDevice, tag: u32) -> Result<(u64, u32), ()> {
        let mut response10 = [0u8; 8];
        let _ = Self::execute_scsi_command_inner(
            device,
            tag,
            &[SCSI_READ_CAPACITY_10, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            response10.as_mut_ptr() as usize,
            response10.len(),
            true,
        )?;
        let last_lba10 = u32::from_be_bytes([response10[0], response10[1], response10[2], response10[3]]);
        let block_size = u32::from_be_bytes([response10[4], response10[5], response10[6], response10[7]]);
        if last_lba10 != u32::MAX {
            return Ok((last_lba10 as u64, block_size));
        }

        let mut response16 = [0u8; 32];
        let mut cdb = [0u8; 16];
        cdb[0] = SCSI_SERVICE_ACTION_IN_16;
        cdb[1] = SCSI_SAI_READ_CAPACITY_16;
        cdb[13] = response16.len() as u8;
        let _ = Self::execute_scsi_command_inner(
            device,
            tag + 1,
            &cdb,
            response16.as_mut_ptr() as usize,
            response16.len(),
            true,
        )?;
        let last_lba = u64::from_be_bytes([
            response16[0], response16[1], response16[2], response16[3], response16[4], response16[5], response16[6], response16[7],
        ]);
        let block_size = u32::from_be_bytes([response16[8], response16[9], response16[10], response16[11]]);
        Ok((last_lba, block_size))
    }

    fn execute_scsi_command(
        &mut self,
        cdb: &[u8],
        buffer_address: usize,
        transfer_length: usize,
        direction_in: bool,
    ) -> Result<usize, ()> {
        let tag = self.tag;
        self.tag = self.tag.wrapping_add(1);
        Self::execute_scsi_command_inner(
            &mut self.device,
            tag,
            cdb,
            buffer_address,
            transfer_length,
            direction_in,
        )
    }

    fn execute_scsi_command_inner(
        device: &mut XhciMassStorageDevice,
        tag: u32,
        cdb: &[u8],
        buffer_address: usize,
        transfer_length: usize,
        direction_in: bool,
    ) -> Result<usize, ()> {
        if cdb.is_empty() || cdb.len() > 16 {
            return Err(());
        }

        let mut cbw = [0u8; 31];
        cbw[0..4].copy_from_slice(&CBW_SIGNATURE.to_le_bytes());
        cbw[4..8].copy_from_slice(&tag.to_le_bytes());
        cbw[8..12].copy_from_slice(&(transfer_length as u32).to_le_bytes());
        cbw[12] = if direction_in { BOT_FLAG_IN } else { 0 };
        cbw[13] = BOT_LUN;
        cbw[14] = cdb.len() as u8;
        cbw[15..15 + cdb.len()].copy_from_slice(cdb);
        let _ = device.bulk_transfer(device.bulk_out_endpoint(), cbw.as_ptr() as usize, cbw.len())?;

        let transferred = if transfer_length != 0 {
            device.bulk_transfer(
                if direction_in {
                    device.bulk_in_endpoint()
                } else {
                    device.bulk_out_endpoint()
                },
                buffer_address,
                transfer_length,
            )?
        } else {
            0
        };

        let mut csw = [0u8; 13];
        let csw_len = device.bulk_transfer(device.bulk_in_endpoint(), csw.as_mut_ptr() as usize, csw.len())?;
        if csw_len != csw.len() {
            println!("USB BOT: short CSW ({csw_len})");
            return Err(());
        }
        let signature = u32::from_le_bytes([csw[0], csw[1], csw[2], csw[3]]);
        let returned_tag = u32::from_le_bytes([csw[4], csw[5], csw[6], csw[7]]);
        let residue = u32::from_le_bytes([csw[8], csw[9], csw[10], csw[11]]);
        let status = csw[12];
        if signature != CSW_SIGNATURE || returned_tag != tag {
            println!("USB BOT: invalid CSW signature/tag");
            return Err(());
        }
        if status != 0 {
            let _ = Self::request_sense_inner(device, tag.wrapping_add(0x1000));
            println!("USB BOT: command failed status={status} residue={residue}");
            return Err(());
        }
        Ok(transferred.saturating_sub(residue as usize))
    }

    fn max_blocks_per_command(&self) -> u64 {
        (u16::MAX as u64).max(1)
    }

}

impl BlockDevice for UsbMassStorage {
    fn read(&mut self, buffer_address: usize, block_address: u64, length: u64) -> Result<(), ()> {
        if (block_address % self.block_size as u64) != 0 || (length % self.block_size as u64) != 0 {
            println!(
                "Block Address({:#X}) and Length({:#X}) must be {}Byte-Aligned.",
                block_address,
                length,
                self.block_size
            );
            return Err(());
        }
        let mut current_lba = block_address / self.block_size as u64;
        let total_blocks = length / self.block_size as u64;
        let mut remaining_blocks = total_blocks;
        let mut buffer = buffer_address;
        while remaining_blocks > 0 {
            let blocks = remaining_blocks.min(self.max_blocks_per_command());
            if blocks != 0 && current_lba + blocks - 1 > self.last_lba {
                println!(
                    "USB BOT: read past end of device (last_lba={:#X}, request_end={:#X})",
                    self.last_lba,
                    current_lba + blocks - 1
                );
                return Err(());
            }
            let transfer_bytes = (blocks * self.block_size as u64) as usize;
            let mut cdb = [0u8; 10];
            cdb[0] = SCSI_READ_10;
            cdb[2..6].copy_from_slice(&(current_lba as u32).to_be_bytes());
            cdb[7..9].copy_from_slice(&(blocks as u16).to_be_bytes());
            let transferred = self.execute_scsi_command(&cdb, buffer, transfer_bytes, true)?;
            if transferred != transfer_bytes {
                println!("USB BOT: READ(10) transferred {transferred} of {transfer_bytes}");
                return Err(());
            }
            current_lba += blocks;
            remaining_blocks -= blocks;
            buffer += transfer_bytes;
        }
        Ok(())
    }

    fn write(&mut self, buffer_address: usize, block_address: u64, length: u64) -> Result<(), ()> {
        if (block_address % self.block_size as u64) != 0 || (length % self.block_size as u64) != 0 {
            println!(
                "Block Address({:#X}) and Length({:#X}) must be {}Byte-Aligned.",
                block_address,
                length,
                self.block_size
            );
            return Err(());
        }
        let mut current_lba = block_address / self.block_size as u64;
        let total_blocks = length / self.block_size as u64;
        let mut remaining_blocks = total_blocks;
        let mut buffer = buffer_address;
        while remaining_blocks > 0 {
            let blocks = remaining_blocks.min(self.max_blocks_per_command());
            if blocks != 0 && current_lba + blocks - 1 > self.last_lba {
                println!(
                    "USB BOT: write past end of device (last_lba={:#X}, request_end={:#X})",
                    self.last_lba,
                    current_lba + blocks - 1
                );
                return Err(());
            }
            let transfer_bytes = (blocks * self.block_size as u64) as usize;
            let mut cdb = [0u8; 10];
            cdb[0] = SCSI_WRITE_10;
            cdb[2..6].copy_from_slice(&(current_lba as u32).to_be_bytes());
            cdb[7..9].copy_from_slice(&(blocks as u16).to_be_bytes());
            let transferred = self.execute_scsi_command(&cdb, buffer, transfer_bytes, false)?;
            if transferred != transfer_bytes {
                println!("USB BOT: WRITE(10) transferred {transferred} of {transfer_bytes}");
                return Err(());
            }
            current_lba += blocks;
            remaining_blocks -= blocks;
            buffer += transfer_bytes;
        }
        Ok(())
    }
}
