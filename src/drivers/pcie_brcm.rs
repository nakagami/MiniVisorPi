//!
//! Broadcom BCM2711 PCIe root-complex driver
//!
//! Provides just enough of the BCM2711 "brcmstb" PCIe host bridge to reach the
//! Raspberry Pi 4's fixed downstream VL805 xHCI controller. This is a
//! from-scratch bring-up driver that has **not** been validated on physical
//! hardware yet: config-space access patterns, register offsets, and DTB-driven
//! address translation follow the Raspberry Pi 4's own U-Boot driver and DTB,
//! but real-hardware testing is still required before relying on it.
//!

use crate::dtb::{Dtb, DtbNode};
use core::ptr::{read_volatile, write_volatile};

const PCIE_MISC_PCIE_STATUS: usize = 0x4068;
const STATUS_PCIE_PHYLINKUP_MASK: u32 = 0x10;
const STATUS_PCIE_DL_ACTIVE_MASK: u32 = 0x20;
const PCIE_EXT_CFG_DATA: usize = 0x8000;
const PCIE_EXT_CFG_INDEX: usize = 0x9000;

const PCI_CONFIG_VENDOR_DEVICE_ID: usize = 0x00;
const PCI_CONFIG_COMMAND_STATUS: usize = 0x04;
const PCI_CONFIG_CLASS_REVISION: usize = 0x08;
const PCI_CONFIG_BAR0: usize = 0x10;

const PCI_COMMAND_MEMORY_SPACE_ENABLE: u16 = 1 << 1;
const PCI_COMMAND_BUS_MASTER_ENABLE: u16 = 1 << 2;

const PCI_CLASS_XHCI: u32 = 0x0C03_30;
const PCI_VENDOR_INVALID: u16 = 0xFFFF;

const DEFAULT_PCI_CHILD_BASE: u64 = 0xF800_0000;
const DEFAULT_PCI_CHILD_SIZE: u64 = 0x0400_0000;
const DEFAULT_CPU_PARENT_BASE: u64 = 0x0000_0006_0000_0000;

#[derive(Clone, Copy)]
pub struct XhciPciDevice {
    pub vendor_id: u16,
    pub device_id: u16,
    pub cpu_mmio_base: usize,
}

pub struct PcieBrcm {
    base_address: usize,
    child_mmio_base: u64,
    parent_mmio_base: u64,
    child_mmio_size: u64,
}

impl PcieBrcm {
    pub fn new(dtb: &Dtb, node: &DtbNode) -> Result<Self, ()> {
        let Some((base_address, _)) = dtb.read_reg_property(node, 0) else {
            println!("PCIe: missing reg property");
            return Err(());
        };
        let base_address = dtb.translate_soc_address(base_address);
        let (child_mmio_base, parent_mmio_base, child_mmio_size) =
            Self::read_mmio_window_from_dtb(dtb, node).unwrap_or((
                DEFAULT_PCI_CHILD_BASE,
                DEFAULT_CPU_PARENT_BASE,
                DEFAULT_PCI_CHILD_SIZE,
            ));
        Ok(Self {
            base_address,
            child_mmio_base,
            parent_mmio_base,
            child_mmio_size,
        })
    }

    pub fn is_link_up(&self) -> bool {
        let status = self.read32(self.base_address + PCIE_MISC_PCIE_STATUS);
        (status & (STATUS_PCIE_PHYLINKUP_MASK | STATUS_PCIE_DL_ACTIVE_MASK))
            == (STATUS_PCIE_PHYLINKUP_MASK | STATUS_PCIE_DL_ACTIVE_MASK)
    }

    pub fn find_xhci_device(&self) -> Option<XhciPciDevice> {
        let vendor_device = self.read_config_dword(1, 0, 0, PCI_CONFIG_VENDOR_DEVICE_ID)?;
        let vendor_id = (vendor_device & 0xFFFF) as u16;
        let device_id = (vendor_device >> 16) as u16;
        if vendor_id == PCI_VENDOR_INVALID {
            return None;
        }

        let class_revision = self.read_config_dword(1, 0, 0, PCI_CONFIG_CLASS_REVISION)?;
        let class_code = class_revision >> 8;
        if class_code != PCI_CLASS_XHCI {
            println!("PCIe: downstream device is class {class_code:06X}, not xHCI");
            return None;
        }

        self.enable_memory_and_bus_master(1, 0, 0).ok()?;
        let cpu_mmio_base = self.translate_bar_to_cpu(self.read_bar_address()?)?;

        Some(XhciPciDevice {
            vendor_id,
            device_id,
            cpu_mmio_base,
        })
    }

    fn read_bar_address(&self) -> Option<u64> {
        let bar0 = self.read_config_dword(1, 0, 0, PCI_CONFIG_BAR0)?;
        if (bar0 & 0x1) != 0 {
            println!("PCIe: xHCI BAR0 is I/O space, unsupported");
            return None;
        }
        let low = (bar0 & !0xF) as u64;
        let bar_type = (bar0 >> 1) & 0x3;
        if bar_type == 0x2 {
            let bar1 = self.read_config_dword(1, 0, 0, PCI_CONFIG_BAR0 + 4)?;
            Some(((bar1 as u64) << 32) | low)
        } else {
            Some(low)
        }
    }

    fn enable_memory_and_bus_master(&self, bus: u8, dev: u8, func: u8) -> Result<(), ()> {
        let command_status = self
            .read_config_dword(bus, dev, func, PCI_CONFIG_COMMAND_STATUS)
            .ok_or(())?;
        let mut command = (command_status & 0xFFFF) as u16;
        command |= PCI_COMMAND_MEMORY_SPACE_ENABLE | PCI_COMMAND_BUS_MASTER_ENABLE;
        let updated = (command_status & !0xFFFF) | (command as u32);
        self.write_config_dword(bus, dev, func, PCI_CONFIG_COMMAND_STATUS, updated)
    }

    fn read_config_dword(&self, bus: u8, dev: u8, func: u8, offset: usize) -> Option<u32> {
        if (offset & 0x3) != 0 {
            return None;
        }
        if bus == 0 {
            return Some(self.read32(self.base_address + offset));
        }
        if bus != 1 || dev != 0 || func != 0 {
            return None;
        }
        let index = ((bus as u32) << 20) | ((dev as u32) << 15) | ((func as u32) << 12);
        self.write32(self.base_address + PCIE_EXT_CFG_INDEX, index);
        Some(self.read32(self.base_address + PCIE_EXT_CFG_DATA + offset))
    }

    fn write_config_dword(
        &self,
        bus: u8,
        dev: u8,
        func: u8,
        offset: usize,
        value: u32,
    ) -> Result<(), ()> {
        if (offset & 0x3) != 0 {
            return Err(());
        }
        if bus == 0 {
            self.write32(self.base_address + offset, value);
            return Ok(());
        }
        if bus != 1 || dev != 0 || func != 0 {
            return Err(());
        }
        let index = ((bus as u32) << 20) | ((dev as u32) << 15) | ((func as u32) << 12);
        self.write32(self.base_address + PCIE_EXT_CFG_INDEX, index);
        self.write32(self.base_address + PCIE_EXT_CFG_DATA + offset, value);
        Ok(())
    }

    fn translate_bar_to_cpu(&self, bar: u64) -> Option<usize> {
        if bar < self.child_mmio_base || bar >= self.child_mmio_base + self.child_mmio_size {
            println!(
                "PCIe: BAR {bar:#X} is outside MMIO window {:#X}..{:#X}",
                self.child_mmio_base,
                self.child_mmio_base + self.child_mmio_size
            );
            return None;
        }
        usize::try_from(self.parent_mmio_base + (bar - self.child_mmio_base)).ok()
    }

    fn read_mmio_window_from_dtb(dtb: &Dtb, node: &DtbNode) -> Option<(u64, u64, u64)> {
        let ranges = dtb.get_property(node, b"ranges")?;
        let cells = dtb.read_property_as_u32_array(&ranges);
        let child_cells = dtb
            .get_property(node, b"#address-cells")
            .and_then(|p| dtb.read_property_as_u32(&p))
            .unwrap_or(3) as usize;
        let parent_cells = node.address_cells() as usize;
        let size_cells = dtb
            .get_property(node, b"#size-cells")
            .and_then(|p| dtb.read_property_as_u32(&p))
            .unwrap_or(2) as usize;
        let entry_cells = child_cells + parent_cells + size_cells;
        if entry_cells == 0 || (cells.len() % entry_cells) != 0 {
            return None;
        }
        for entry in cells.chunks_exact(entry_cells) {
            if child_cells < 3 {
                continue;
            }
            let flags = u32::from_be(entry[0]);
            let space_code = (flags >> 24) & 0x3;
            if space_code != 0x2 {
                continue;
            }
            let child = Self::u64_from_cells_be(&entry[1..child_cells]);
            let parent = Self::u64_from_cells_be(&entry[child_cells..child_cells + parent_cells]);
            let size = Self::u64_from_cells_be(&entry[child_cells + parent_cells..entry_cells]);
            return Some((child, parent, size));
        }
        None
    }

    fn u64_from_cells_be(cells: &[u32]) -> u64 {
        let mut value = 0u64;
        for cell in cells {
            value = (value << 32) | (u32::from_be(*cell) as u64);
        }
        value
    }

    fn read32(&self, address: usize) -> u32 {
        unsafe { read_volatile(address as *const u32) }
    }

    fn write32(&self, address: usize, value: u32) {
        unsafe { write_volatile(address as *mut u32, value) };
    }
}
