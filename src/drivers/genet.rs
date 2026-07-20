//!
//! Broadcom GENET (BCM2711, v5) driver
//!
//! This driver implements a minimal single-queue RX/TX DMA data path used as
//! the physical backend for the guest-facing virtio-net emulation on
//! Raspberry Pi 4.
//!

use crate::asm;
use core::mem::size_of;
use core::ptr::{copy_nonoverlapping, read_volatile, write_volatile};

/* Register definitions (derived from U-Boot/Linux bcmgenet drivers) */
const SYS_PORT_CTRL: usize = 0x0004;
const SYS_RBUF_FLUSH_CTRL: usize = 0x0008;
const PORT_MODE_EXT_GPHY: u32 = 3;

const GENET_EXT_OFF: usize = 0x0080;
const EXT_RGMII_OOB_CTRL: usize = GENET_EXT_OFF + 0x0C;
const RGMII_LINK: u32 = 1 << 4;
const OOB_DISABLE: u32 = 1 << 5;
const RGMII_MODE_EN: u32 = 1 << 6;
const ID_MODE_DIS: u32 = 1 << 16;

const GENET_RBUF_OFF: usize = 0x0300;
const RBUF_CTRL: usize = GENET_RBUF_OFF;
const RBUF_ALIGN_2B: u32 = 1 << 1;
const RBUF_TBUF_SIZE_CTRL: usize = GENET_RBUF_OFF + 0xB4;

const GENET_UMAC_OFF: usize = 0x0800;
const UMAC_CMD: usize = GENET_UMAC_OFF + 0x008;
const UMAC_MAC0: usize = GENET_UMAC_OFF + 0x00C;
const UMAC_MAC1: usize = GENET_UMAC_OFF + 0x010;
const UMAC_MAX_FRAME_LEN: usize = GENET_UMAC_OFF + 0x014;
const UMAC_MODE: usize = GENET_UMAC_OFF + 0x044;
const UMAC_MIB_CTRL: usize = GENET_UMAC_OFF + 0x580;
const UMAC_MDIO_CMD: usize = GENET_UMAC_OFF + 0x614;
const UMAC_TX_FLUSH: usize = GENET_UMAC_OFF + 0x334;

const CMD_TX_EN: u32 = 1 << 0;
const CMD_RX_EN: u32 = 1 << 1;
const CMD_SPEED_SHIFT: u32 = 2;
const CMD_SPEED_10: u32 = 0;
const CMD_SPEED_100: u32 = 1;
const CMD_SPEED_1000: u32 = 2;
const CMD_HD_EN: u32 = 1 << 10;
const CMD_SW_RESET: u32 = 1 << 13;
const CMD_LCL_LOOP_EN: u32 = 1 << 15;

const MIB_RESET_RX: u32 = 1 << 0;
const MIB_RESET_RUNT: u32 = 1 << 1;
const MIB_RESET_TX: u32 = 1 << 2;

const MODE_LINK_STATUS: u32 = 1 << 5;

const MDIO_START_BUSY: u32 = 1 << 29;
const MDIO_READ_FAIL: u32 = 1 << 28;
const MDIO_RD: u32 = 2 << 26;
const MDIO_PMD_SHIFT: u32 = 21;
const MDIO_PMD_MASK: u32 = 0x1F;
const MDIO_REG_SHIFT: u32 = 16;
const MDIO_REG_MASK: u32 = 0x1F;

/* Standard MII registers used to resolve the speed/duplex actually
 * negotiated with the link partner, instead of assuming a fixed speed. */
const MII_ADVERTISE: u8 = 4;
const MII_LPA: u8 = 5;
const MII_CTRL1000: u8 = 9;
const MII_STAT1000: u8 = 10;

const BMSR_ANEGCOMPLETE: u16 = 1 << 5;

const ADVERTISE_10FULL: u16 = 1 << 6;
const ADVERTISE_100HALF: u16 = 1 << 7;
const ADVERTISE_100FULL: u16 = 1 << 8;

const ADVERTISE_1000FULL: u16 = 1 << 9;
const ADVERTISE_1000HALF: u16 = 1 << 8;
const LPA_1000HALF: u16 = 1 << 10;
const LPA_1000FULL: u16 = 1 << 11;

/* DMA layout */
const DEFAULT_Q: usize = 0x10;
const DMA_RING_SIZE: usize = 0x40;
const DMA_RINGS_SIZE: usize = DMA_RING_SIZE * (DEFAULT_Q + 1);

const DMA_DESC_LENGTH_STATUS: usize = 0x00;
const DMA_DESC_ADDRESS_LO: usize = 0x04;
const DMA_DESC_ADDRESS_HI: usize = 0x08;
const DMA_DESC_SIZE: usize = 12;

const DMA_EN: u32 = 1 << 0;
const DMA_RING_BUF_EN_SHIFT: u32 = 1;
const DMA_BUFLENGTH_MASK: u32 = 0x0FFF;
const DMA_BUFLENGTH_SHIFT: u32 = 16;
const DMA_RING_SIZE_SHIFT: u32 = 16;
const DMA_OWN: u32 = 0x8000;
const DMA_EOP: u32 = 0x4000;
const DMA_SOP: u32 = 0x2000;
const DMA_TX_APPEND_CRC: u32 = 0x0040;
const DMA_TX_QTAG_SHIFT: u32 = 7;
const DMA_MAX_BURST_LENGTH: u32 = 0x8;

const GENET_RX_OFF: usize = 0x2000;
const GENET_RDMA_REG_OFF: usize = GENET_RX_OFF + (NUMBER_OF_DESCRIPTORS * DMA_DESC_SIZE);
const GENET_TX_OFF: usize = 0x4000;
const GENET_TDMA_REG_OFF: usize = GENET_TX_OFF + (NUMBER_OF_DESCRIPTORS * DMA_DESC_SIZE);

const TDMA_RING_REG_BASE: usize = GENET_TDMA_REG_OFF + (DEFAULT_Q * DMA_RING_SIZE);
const TDMA_READ_PTR: usize = TDMA_RING_REG_BASE;
const TDMA_CONS_INDEX: usize = TDMA_RING_REG_BASE + 0x08;
const TDMA_PROD_INDEX: usize = TDMA_RING_REG_BASE + 0x0C;
const TDMA_MBUF_DONE_THRESH: usize = TDMA_RING_REG_BASE + 0x24;
const TDMA_FLOW_PERIOD: usize = TDMA_RING_REG_BASE + 0x28;
const TDMA_WRITE_PTR: usize = TDMA_RING_REG_BASE + 0x2C;

const RDMA_RING_REG_BASE: usize = GENET_RDMA_REG_OFF + (DEFAULT_Q * DMA_RING_SIZE);
const RDMA_WRITE_PTR: usize = RDMA_RING_REG_BASE;
const RDMA_PROD_INDEX: usize = RDMA_RING_REG_BASE + 0x08;
const RDMA_CONS_INDEX: usize = RDMA_RING_REG_BASE + 0x0C;
const RDMA_XON_XOFF_THRESH: usize = RDMA_RING_REG_BASE + 0x28;
const RDMA_READ_PTR: usize = RDMA_RING_REG_BASE + 0x2C;

const DMA_RING_BUF_SIZE: usize = 0x10;
const DMA_START_ADDR: usize = 0x14;
const DMA_END_ADDR: usize = 0x1C;
const DMA_SCB_BURST_SIZE: usize = 0x0C;
const DMA_CTRL: usize = 0x04;
const DMA_RING_CFG: usize = 0x00;

const TDMA_REG_BASE: usize = GENET_TDMA_REG_OFF + DMA_RINGS_SIZE;
const RDMA_REG_BASE: usize = GENET_RDMA_REG_OFF + DMA_RINGS_SIZE;

const DMA_FC_THRESH_HI: u32 = (NUMBER_OF_DESCRIPTORS as u32) >> 4;
const DMA_FC_THRESH_LO: u32 = 5;
const DMA_FC_THRESH_VALUE: u32 = (DMA_FC_THRESH_LO << 16) | DMA_FC_THRESH_HI;

/* PHY */
const MII_BMSR: u8 = 1;
const MII_PHYSID1: u8 = 2;
const MII_PHYSID2: u8 = 3;
const BMSR_LSTATUS: u16 = 1 << 2;

const PHY_ADDRESS_MAX: u8 = 32;
const MDIO_POLL_TIMEOUT: usize = 10_000;
const DMA_POLL_TIMEOUT: usize = 1_000_000;

const NUMBER_OF_DESCRIPTORS: usize = 256;
const RX_BUFFER_LENGTH: usize = 2048;
const RX_BUF_OFFSET: usize = 2;
const ENET_MAX_MTU_SIZE: u32 = 1536;

const MIN_REGISTER_RANGE: usize = 0x10000;

#[derive(Copy, Clone)]
pub struct LinkStatus {
    pub phy_link: bool,
    pub mac_link: bool,
}

impl LinkStatus {
    pub fn is_up(&self) -> bool {
        self.phy_link || self.mac_link
    }
}

pub struct Genet {
    base_address: usize,
    phy_addr: u8,
    phy_id1: u16,
    phy_id2: u16,
    tx_index: u16,
    rx_index: u16,
    c_index: u16,
    rx_buffers: usize,
    mac: [u8; 6],
    data_path_ready: bool,
}

impl Genet {
    pub fn new(base_address: usize, range: usize) -> Result<Self, ()> {
        if range < MIN_REGISTER_RANGE {
            println!("GENET: MMIO range is too small ({range:#X})");
            return Err(());
        }
        let Some((phy_addr, phy_id1, phy_id2)) = Self::detect_phy(base_address) else {
            println!("GENET: no PHY was detected on MDIO bus");
            return Err(());
        };

        let rx_buffers = crate::allocate_pages(
            (NUMBER_OF_DESCRIPTORS * RX_BUFFER_LENGTH).div_ceil(crate::paging::PAGE_SIZE),
            crate::paging::PAGE_SHIFT,
        )
        .map_err(|_| ())?;
        let mut net = Self {
            base_address,
            phy_addr,
            phy_id1,
            phy_id2,
            tx_index: 0,
            rx_index: 0,
            c_index: 0,
            rx_buffers,
            mac: [0x52, 0x54, 0x00, 0x12, 0x34, 0x56],
            data_path_ready: false,
        };
        net.maybe_read_mac_address();
        Ok(net)
    }

    pub fn get_base_address(&self) -> usize {
        self.base_address
    }

    pub fn get_phy_address(&self) -> u8 {
        self.phy_addr
    }

    pub fn get_phy_id(&self) -> (u16, u16) {
        (self.phy_id1, self.phy_id2)
    }

    pub fn get_mac_address(&self) -> [u8; 6] {
        self.mac
    }

    pub fn read_link_status(&self) -> Result<LinkStatus, ()> {
        /* BMSR link bit is latch-low, so read twice and OR the result. */
        let bmsr_first = self.mdio_read(self.phy_addr, MII_BMSR)?;
        let bmsr_second = self.mdio_read(self.phy_addr, MII_BMSR)?;
        Ok(LinkStatus {
            phy_link: ((bmsr_first | bmsr_second) & BMSR_LSTATUS) != 0,
            mac_link: (self.read_register(UMAC_MODE) & MODE_LINK_STATUS) != 0,
        })
    }

    pub fn wait_link_up(&self, timeout_ms: u64) -> Result<LinkStatus, ()> {
        let frequency = asm::get_cntfrq_el0();
        if frequency == 0 {
            return self.read_link_status();
        }
        let interval_ticks = (frequency / 100).max(1); /* 10ms */
        let timeout_ticks = frequency.saturating_mul(timeout_ms) / 1000;
        let start = asm::get_cntpct_el0();
        let mut next_poll = start;
        let mut last_status = self.read_link_status()?;

        loop {
            let now = asm::get_cntpct_el0();
            if now.wrapping_sub(start) >= timeout_ticks {
                return Ok(last_status);
            }
            if now.wrapping_sub(next_poll) < interval_ticks {
                core::hint::spin_loop();
                continue;
            }
            next_poll = now;
            last_status = self.read_link_status()?;
            if last_status.is_up() {
                return Ok(last_status);
            }
        }
    }

    pub fn send(&mut self, buffer_address: usize, length: usize) -> Result<(), ()> {
        self.ensure_data_path_ready()?;
        let descriptor_base =
            self.base_address + GENET_TX_OFF + (self.tx_index as usize) * DMA_DESC_SIZE;
        let length_status = ((length as u32) << DMA_BUFLENGTH_SHIFT)
            | (0x3F << DMA_TX_QTAG_SHIFT)
            | DMA_TX_APPEND_CRC
            | DMA_SOP
            | DMA_EOP;
        unsafe { asm::clean_dcache_range(buffer_address, length) };
        self.write_descriptor_address(descriptor_base, buffer_address);
        self.write_register(descriptor_base + DMA_DESC_LENGTH_STATUS, length_status);

        self.tx_index = self.tx_index.wrapping_add(1);
        if (self.tx_index as usize) >= NUMBER_OF_DESCRIPTORS {
            self.tx_index = 0;
        }
        let target_prod = (self.read_register(TDMA_PROD_INDEX) & 0xFFFF).wrapping_add(1);
        self.write_register(TDMA_PROD_INDEX, target_prod);

        for _ in 0..DMA_POLL_TIMEOUT {
            let cons = self.read_register(TDMA_CONS_INDEX) & 0xFFFF;
            if cons >= target_prod {
                return Ok(());
            }
            core::hint::spin_loop();
        }
        println!(
            "GENET: TX timed out waiting for completion (target_prod={target_prod:#X} \
             cons={:#X} tdma_ctrl={:#X})",
            self.read_register(TDMA_CONS_INDEX) & 0xFFFF,
            self.read_register(TDMA_REG_BASE + DMA_CTRL)
        );
        Err(())
    }

    pub fn poll_rx(&mut self, buffer: &mut [u8]) -> Option<usize> {
        /* Avoid first-time DMA setup from the WFI trap path, where a long
         * operation would stall guest progress. The data path is armed on the
         * first TX (`send`) instead. */
        if !self.data_path_ready {
            return None;
        }
        let producer_index = self.read_register(RDMA_PROD_INDEX) & 0xFFFF;
        if producer_index == self.c_index as u32 {
            return None;
        }
        if producer_index.wrapping_sub(self.c_index as u32) > NUMBER_OF_DESCRIPTORS as u32 {
            return None;
        }
        let descriptor_base =
            self.base_address + GENET_RX_OFF + (self.rx_index as usize) * DMA_DESC_SIZE;
        let length_status = self.read_register(descriptor_base + DMA_DESC_LENGTH_STATUS);
        if (length_status & DMA_OWN) != 0 {
            return None;
        }
        let total_length =
            ((length_status >> DMA_BUFLENGTH_SHIFT) & DMA_BUFLENGTH_MASK) as usize;
        if total_length == 0 || total_length > RX_BUFFER_LENGTH {
            return None;
        }
        let data_length = total_length.saturating_sub(RX_BUF_OFFSET);

        let packet_address = self.read_descriptor_address(descriptor_base);
        unsafe { asm::invalidate_dcache_range(packet_address, total_length) };
        let copy_length = data_length.min(buffer.len());
        unsafe {
            copy_nonoverlapping(
                (packet_address + RX_BUF_OFFSET) as *const u8,
                buffer.as_mut_ptr(),
                copy_length,
            );
        }

        self.write_register(
            descriptor_base + DMA_DESC_LENGTH_STATUS,
            ((RX_BUFFER_LENGTH as u32) << DMA_BUFLENGTH_SHIFT) | DMA_OWN,
        );
        self.c_index = self.c_index.wrapping_add(1);
        self.write_register(RDMA_CONS_INDEX, self.c_index as u32);
        self.rx_index = self.rx_index.wrapping_add(1);
        if (self.rx_index as usize) >= NUMBER_OF_DESCRIPTORS {
            self.rx_index = 0;
        }
        println!("GENET: RX {copy_length} bytes (producer_index={producer_index})");
        Some(copy_length)
    }

    fn detect_phy(base_address: usize) -> Option<(u8, u16, u16)> {
        if let Some(found) = Self::try_probe_phy(base_address, 1) {
            return Some(found);
        }
        for phy_addr in 0..PHY_ADDRESS_MAX {
            if phy_addr == 1 {
                continue;
            }
            if let Some(found) = Self::try_probe_phy(base_address, phy_addr) {
                return Some(found);
            }
        }
        None
    }

    fn try_probe_phy(base_address: usize, phy_addr: u8) -> Option<(u8, u16, u16)> {
        /* BCM7xxx integrated PHYs sometimes need a dummy first read. */
        let _ = Self::mdio_read_with_base(base_address, phy_addr, MII_BMSR);
        let Ok(phy_id1) = Self::mdio_read_with_base(base_address, phy_addr, MII_PHYSID1) else {
            return None;
        };
        let Ok(phy_id2) = Self::mdio_read_with_base(base_address, phy_addr, MII_PHYSID2) else {
            return None;
        };
        if phy_id1 == 0 || phy_id1 == u16::MAX || phy_id2 == 0 || phy_id2 == u16::MAX {
            return None;
        }
        Some((phy_addr, phy_id1, phy_id2))
    }

    fn ensure_data_path_ready(&mut self) -> Result<(), ()> {
        if self.data_path_ready {
            return Ok(());
        }
        self.umac_reset();
        self.setup_rgmii_mode();
        self.disable_dma();
        self.rx_ring_init();
        self.rx_descs_init();
        self.tx_ring_init();
        self.enable_dma();
        self.enable_rx_tx();
        self.data_path_ready = true;
        let (speed, half_duplex) = self.resolve_negotiated_speed();
        println!(
            "GENET: data path armed (speed_code={speed} half_duplex={half_duplex} \
             tx_index={} rx_index={} c_index={})",
            self.tx_index, self.rx_index, self.c_index
        );
        Ok(())
    }

    fn maybe_read_mac_address(&mut self) {
        let mac0 = self.read_register(UMAC_MAC0);
        let mac1 = self.read_register(UMAC_MAC1);
        let mac = [
            ((mac0 >> 24) & 0xFF) as u8,
            ((mac0 >> 16) & 0xFF) as u8,
            ((mac0 >> 8) & 0xFF) as u8,
            (mac0 & 0xFF) as u8,
            ((mac1 >> 8) & 0xFF) as u8,
            (mac1 & 0xFF) as u8,
        ];
        if mac != [0; 6] {
            self.mac = mac;
        }
    }

    fn mdio_read(&self, phy_addr: u8, reg: u8) -> Result<u16, ()> {
        Self::mdio_read_with_base(self.base_address, phy_addr, reg)
    }

    fn mdio_read_with_base(base_address: usize, phy_addr: u8, reg: u8) -> Result<u16, ()> {
        let command = MDIO_RD
            | (((phy_addr as u32) & MDIO_PMD_MASK) << MDIO_PMD_SHIFT)
            | (((reg as u32) & MDIO_REG_MASK) << MDIO_REG_SHIFT);
        Self::write_register_with_base(base_address, UMAC_MDIO_CMD, command);
        Self::write_register_with_base(base_address, UMAC_MDIO_CMD, command | MDIO_START_BUSY);

        if let Some(status) = Self::wait_mdio_complete(base_address) {
            if (status & MDIO_READ_FAIL) != 0 {
                return Err(());
            }
            return Ok((status & 0xFFFF) as u16);
        }
        Err(())
    }

    fn wait_mdio_complete(base_address: usize) -> Option<u32> {
        let frequency = asm::get_cntfrq_el0();
        if frequency != 0 {
            let timeout_ticks = (frequency / 200).max(1); /* 5ms */
            let start = asm::get_cntpct_el0();
            loop {
                let status = Self::read_register_with_base(base_address, UMAC_MDIO_CMD);
                if (status & MDIO_START_BUSY) == 0 {
                    return Some(status);
                }
                if asm::get_cntpct_el0().wrapping_sub(start) >= timeout_ticks {
                    return None;
                }
                core::hint::spin_loop();
            }
        }
        for _ in 0..MDIO_POLL_TIMEOUT {
            /* BCM7xxx integrated PHYs sometimes need a dummy first read. */
            let status = Self::read_register_with_base(base_address, UMAC_MDIO_CMD);
            if (status & MDIO_START_BUSY) == 0 {
                return Some(status);
            }
            core::hint::spin_loop();
        }
        None
    }

    fn umac_reset(&self) {
        self.write_register(SYS_PORT_CTRL, PORT_MODE_EXT_GPHY);
        self.write_register(SYS_RBUF_FLUSH_CTRL, 0);
        self.write_register(UMAC_CMD, 0);
        self.write_register(UMAC_CMD, CMD_SW_RESET | CMD_LCL_LOOP_EN);
        self.write_register(UMAC_CMD, 0);
        self.write_register(UMAC_MIB_CTRL, MIB_RESET_RX | MIB_RESET_TX | MIB_RESET_RUNT);
        self.write_register(UMAC_MIB_CTRL, 0);
        self.write_register(UMAC_MAX_FRAME_LEN, ENET_MAX_MTU_SIZE);
        self.write_register(
            RBUF_CTRL,
            self.read_register(RBUF_CTRL) | RBUF_ALIGN_2B,
        );
        self.write_register(RBUF_TBUF_SIZE_CTRL, 1);
        /* CMD_SW_RESET above clears the UMAC unicast address filter back to
         * all-zero, which would otherwise make the hardware drop every
         * unicast frame (e.g. a DHCP OFFER/ACK sent directly to our MAC
         * instead of broadcast), leaving only broadcast traffic reaching
         * poll_rx(). Restore the address the constructor captured from
         * firmware (or the fallback default) so unicast reception works
         * once the data path is armed. */
        self.write_hw_addr();
    }

    fn write_hw_addr(&self) {
        let mac = self.mac;
        let mac0 = ((mac[0] as u32) << 24)
            | ((mac[1] as u32) << 16)
            | ((mac[2] as u32) << 8)
            | (mac[3] as u32);
        let mac1 = ((mac[4] as u32) << 8) | (mac[5] as u32);
        self.write_register(UMAC_MAC0, mac0);
        self.write_register(UMAC_MAC1, mac1);
    }

    fn setup_rgmii_mode(&self) {
        let mut reg = self.read_register(EXT_RGMII_OOB_CTRL);
        reg &= !OOB_DISABLE;
        reg |= RGMII_LINK | RGMII_MODE_EN | ID_MODE_DIS;
        self.write_register(EXT_RGMII_OOB_CTRL, reg);
    }

    fn disable_dma(&self) {
        let tdma_ctrl = self.read_register(TDMA_REG_BASE + DMA_CTRL) & !DMA_EN;
        self.write_register(TDMA_REG_BASE + DMA_CTRL, tdma_ctrl);
        let rdma_ctrl = self.read_register(RDMA_REG_BASE + DMA_CTRL) & !DMA_EN;
        self.write_register(RDMA_REG_BASE + DMA_CTRL, rdma_ctrl);
        self.write_register(UMAC_TX_FLUSH, 1);
        self.write_register(UMAC_TX_FLUSH, 0);
    }

    fn enable_dma(&self) {
        let dma_ctrl = (1 << (DEFAULT_Q as u32 + DMA_RING_BUF_EN_SHIFT)) | DMA_EN;
        self.write_register(TDMA_REG_BASE + DMA_CTRL, dma_ctrl);
        self.write_register(RDMA_REG_BASE + DMA_CTRL, dma_ctrl);
    }

    fn rx_descs_init(&self) {
        let length_status = ((RX_BUFFER_LENGTH as u32) << DMA_BUFLENGTH_SHIFT) | DMA_OWN;
        for i in 0..NUMBER_OF_DESCRIPTORS {
            let descriptor_base = self.base_address + GENET_RX_OFF + i * DMA_DESC_SIZE;
            let buffer_address = self.rx_buffers + i * RX_BUFFER_LENGTH;
            self.write_descriptor_address(descriptor_base, buffer_address);
            self.write_register(descriptor_base + DMA_DESC_LENGTH_STATUS, length_status);
            unsafe { asm::clean_dcache_range(buffer_address, RX_BUFFER_LENGTH) };
        }
    }

    fn rx_ring_init(&mut self) {
        self.write_register(RDMA_REG_BASE + DMA_SCB_BURST_SIZE, DMA_MAX_BURST_LENGTH);
        self.write_register(RDMA_RING_REG_BASE + DMA_START_ADDR, 0);
        self.write_register(RDMA_READ_PTR, 0);
        self.write_register(RDMA_WRITE_PTR, 0);
        self.write_register(
            RDMA_RING_REG_BASE + DMA_END_ADDR,
            ((NUMBER_OF_DESCRIPTORS * DMA_DESC_SIZE) / size_of::<u32>() - 1) as u32,
        );
        self.c_index = (self.read_register(RDMA_PROD_INDEX) & 0xFFFF) as u16;
        self.write_register(RDMA_CONS_INDEX, self.c_index as u32);
        self.rx_index = self.c_index & 0xFF;
        self.write_register(
            RDMA_RING_REG_BASE + DMA_RING_BUF_SIZE,
            ((NUMBER_OF_DESCRIPTORS as u32) << DMA_RING_SIZE_SHIFT) | (RX_BUFFER_LENGTH as u32),
        );
        self.write_register(RDMA_XON_XOFF_THRESH, DMA_FC_THRESH_VALUE);
        self.write_register(RDMA_REG_BASE + DMA_RING_CFG, 1 << DEFAULT_Q);
    }

    fn tx_ring_init(&mut self) {
        self.write_register(TDMA_REG_BASE + DMA_SCB_BURST_SIZE, DMA_MAX_BURST_LENGTH);
        self.write_register(TDMA_RING_REG_BASE + DMA_START_ADDR, 0);
        self.write_register(TDMA_READ_PTR, 0);
        self.write_register(TDMA_WRITE_PTR, 0);
        self.write_register(
            TDMA_RING_REG_BASE + DMA_END_ADDR,
            ((NUMBER_OF_DESCRIPTORS * DMA_DESC_SIZE) / size_of::<u32>() - 1) as u32,
        );
        self.tx_index = (self.read_register(TDMA_CONS_INDEX) & 0xFFFF) as u16;
        self.write_register(TDMA_PROD_INDEX, self.tx_index as u32);
        self.tx_index &= 0xFF;
        self.write_register(TDMA_MBUF_DONE_THRESH, 1);
        self.write_register(TDMA_FLOW_PERIOD, 0);
        self.write_register(
            TDMA_RING_REG_BASE + DMA_RING_BUF_SIZE,
            ((NUMBER_OF_DESCRIPTORS as u32) << DMA_RING_SIZE_SHIFT) | (RX_BUFFER_LENGTH as u32),
        );
        self.write_register(TDMA_REG_BASE + DMA_RING_CFG, 1 << DEFAULT_Q);
    }

    fn enable_rx_tx(&self) {
        let (speed, half_duplex) = self.resolve_negotiated_speed();
        let mut cmd = speed << CMD_SPEED_SHIFT;
        if half_duplex {
            cmd |= CMD_HD_EN;
        }
        self.write_register(UMAC_CMD, cmd);
        self.write_register(UMAC_CMD, cmd | CMD_TX_EN | CMD_RX_EN);
    }

    /// Resolves the speed/duplex actually negotiated with the link partner
    /// (mirroring what Linux/U-Boot's phylib does via genphy_read_status),
    /// instead of assuming a fixed speed. Forcing the UMAC to a speed the
    /// PHY didn't actually negotiate silently corrupts every frame on the
    /// wire (TX and RX), which manifests as e.g. DHCP never getting a
    /// reply even though the physical link LEDs are up.
    fn resolve_negotiated_speed(&self) -> (u32, bool) {
        let bmsr = self.mdio_read(self.phy_addr, MII_BMSR).unwrap_or(0);
        if (bmsr & BMSR_ANEGCOMPLETE) == 0 {
            /* Autonegotiation hasn't completed (or is disabled): fall back
             * to the safest common denominator rather than risk a mismatch. */
            return (CMD_SPEED_10, true);
        }

        let gtctl = self.mdio_read(self.phy_addr, MII_CTRL1000).unwrap_or(0);
        let gtsr = self.mdio_read(self.phy_addr, MII_STAT1000).unwrap_or(0);
        if (gtctl & ADVERTISE_1000FULL) != 0 && (gtsr & LPA_1000FULL) != 0 {
            return (CMD_SPEED_1000, false);
        }
        if (gtctl & ADVERTISE_1000HALF) != 0 && (gtsr & LPA_1000HALF) != 0 {
            return (CMD_SPEED_1000, true);
        }

        let advertise = self.mdio_read(self.phy_addr, MII_ADVERTISE).unwrap_or(0);
        let lpa = self.mdio_read(self.phy_addr, MII_LPA).unwrap_or(0);
        let common = advertise & lpa;
        if (common & ADVERTISE_100FULL) != 0 {
            return (CMD_SPEED_100, false);
        }
        if (common & ADVERTISE_100HALF) != 0 {
            return (CMD_SPEED_100, true);
        }
        if (common & ADVERTISE_10FULL) != 0 {
            return (CMD_SPEED_10, false);
        }
        (CMD_SPEED_10, true)
    }

    fn read_descriptor_address(&self, descriptor_base: usize) -> usize {
        let low = self.read_register(descriptor_base + DMA_DESC_ADDRESS_LO) as u64;
        let high = self.read_register(descriptor_base + DMA_DESC_ADDRESS_HI) as u64;
        ((high << 32) | low) as usize
    }

    fn write_descriptor_address(&self, descriptor_base: usize, address: usize) {
        self.write_register(descriptor_base + DMA_DESC_ADDRESS_LO, address as u32);
        self.write_register(
            descriptor_base + DMA_DESC_ADDRESS_HI,
            ((address as u64 >> 32) & 0xFFFF_FFFF) as u32,
        );
    }

    fn read_register(&self, offset: usize) -> u32 {
        Self::read_register_with_base(self.base_address, offset)
    }

    fn write_register(&self, offset: usize, value: u32) {
        Self::write_register_with_base(self.base_address, offset, value)
    }

    fn read_register_with_base(base_address: usize, offset: usize) -> u32 {
        unsafe { read_volatile((base_address + offset) as *const u32) }
    }

    fn write_register_with_base(base_address: usize, offset: usize, value: u32) {
        unsafe { write_volatile((base_address + offset) as *mut u32, value) }
    }
}
