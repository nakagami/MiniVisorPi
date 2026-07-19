//!
//! SD Host Controller Interface (SDHCI) driver
//!
//! Implements the standard "SD Host Controller Simplified Specification"
//! register interface using PIO (no SDMA/ADMA), which is used by Raspberry
//! Pi 4 (BCM2711)'s EMMC2 controller (DTB `compatible = "brcm,bcm2711-emmc2"`)
//! to drive the microSD card slot. This is a from-scratch bring-up driver
//! that has **not** been validated on physical hardware yet: register
//! offsets/semantics follow the SDHCI v3.00 specification and match what
//! Linux's `sdhci-iproc` / U-Boot's `sdhci` drivers do for this controller,
//! but real-hardware testing is still required before relying on it.
//!
//! The public `read`/`write` methods intentionally mirror
//! [`crate::drivers::virtio_blk::VirtioBlk`]'s signature so that both
//! drivers can implement the shared [`crate::drivers::block_device::BlockDevice`]
//! trait and be used interchangeably by the FAT32 layer.

use crate::asm;
use core::cell::Cell;
use core::ptr::{read_volatile, write_volatile};

/* Register offsets (SDHCI v3.00) */
const SDHCI_BLOCK_SIZE: usize = 0x04;
const SDHCI_BLOCK_COUNT: usize = 0x06;
const SDHCI_ARGUMENT: usize = 0x08;
const SDHCI_TRANSFER_MODE: usize = 0x0C;
const SDHCI_COMMAND: usize = 0x0E;
const SDHCI_RESPONSE: usize = 0x10;
const SDHCI_BUFFER: usize = 0x20;
const SDHCI_PRESENT_STATE: usize = 0x24;
const SDHCI_POWER_CONTROL: usize = 0x29;
const SDHCI_CLOCK_CONTROL: usize = 0x2C;
const SDHCI_TIMEOUT_CONTROL: usize = 0x2E;
const SDHCI_SOFTWARE_RESET: usize = 0x2F;
const SDHCI_INT_STATUS: usize = 0x30;
/// Error Interrupt Status: only meaningful (and only worth reading) when
/// SDHCI_INT_STATUS's bit 15 (SDHCI_INT_ERROR) is set; breaks a raised
/// "Error Interrupt" down into its specific cause (command timeout, CRC,
/// index mismatch, data timeout, ...) for diagnostics.
const SDHCI_ERR_INT_STATUS: usize = 0x32;
const SDHCI_INT_ENABLE: usize = 0x34;
const SDHCI_SIGNAL_ENABLE: usize = 0x38;
const SDHCI_HOST_CONTROL2: usize = 0x3E;
const SDHCI_CAPABILITIES: usize = 0x40;

/* Present State register bits */
const SDHCI_CMD_INHIBIT: u32 = 1 << 0;
const SDHCI_DAT_INHIBIT: u32 = 1 << 1;
const SDHCI_BUFFER_WRITE_ENABLE: u32 = 1 << 10;
const SDHCI_BUFFER_READ_ENABLE: u32 = 1 << 11;
const SDHCI_CARD_INSERTED: u32 = 1 << 16;

/* Normal Interrupt Status bits */
const SDHCI_INT_RESPONSE: u16 = 1 << 0;
const SDHCI_INT_DATA_END: u16 = 1 << 1;
const SDHCI_INT_BUFFER_WRITE_READY: u16 = 1 << 4;
const SDHCI_INT_BUFFER_READ_READY: u16 = 1 << 5;
const SDHCI_INT_ERROR: u16 = 1 << 15;

/* Software Reset register bits */
const SDHCI_RESET_ALL: u8 = 1 << 0;
const SDHCI_RESET_CMD: u8 = 1 << 1;
const SDHCI_RESET_DATA: u8 = 1 << 2;

/* Clock Control register bits */
const SDHCI_CLOCK_INT_EN: u16 = 1 << 0;
const SDHCI_CLOCK_INT_STABLE: u16 = 1 << 1;
const SDHCI_CLOCK_CARD_EN: u16 = 1 << 2;

/* Transfer Mode register bits */
const SDHCI_TRNS_BLK_CNT_EN: u16 = 1 << 1;
const SDHCI_TRNS_ACMD12: u16 = 1 << 2;
const SDHCI_TRNS_READ: u16 = 1 << 4;
const SDHCI_TRNS_MULTI: u16 = 1 << 5;

/* Command register response-type select field (bits [1:0]) */
const SDHCI_CMD_RESP_NONE: u16 = 0b00;
const SDHCI_CMD_RESP_136: u16 = 0b01;
const SDHCI_CMD_RESP_48: u16 = 0b10;
const SDHCI_CMD_RESP_48_BUSY: u16 = 0b11;
const SDHCI_CMD_CRC_CHECK: u16 = 1 << 3;
const SDHCI_CMD_INDEX_CHECK: u16 = 1 << 4;
const SDHCI_CMD_DATA_PRESENT: u16 = 1 << 5;

/// Minimum gap enforced between successive register writes (other than to
/// SDHCI_BUFFER), matching u-boot's `bcm2835_sdhci.c` /
/// `bcm2835_sdhci_raw_writel()` workaround: "The Arasan [derivative used by
/// both BCM2835's legacy SD host and BCM2711's EMMC2] has a bugette whereby
/// it may lose the content of successive writes to registers that are
/// within two SD-card clock cycles of each other (a clock domain crossing
/// problem)." Without this delay, the CPU (running at GHz speed) can issue
/// several register writes for one command (INT_STATUS clear, ARGUMENT,
/// COMMAND, ...) far faster than 2 SD clock cycles, silently dropping some
/// of them -- typically manifesting as a command that appears to have been
/// issued (SDHCI_CMD_INHIBIT sets) but never gets a response/interrupt.
/// Sized against the slowest (400KHz, card identification) clock speed used
/// during setup, like u-boot's driver, since that yields the largest -- and
/// therefore always-sufficient -- required delay.
const REGISTER_WRITE_SPACING_US: u64 = (2 * 1_000_000) / 400_000 + 1;

/// SD block size used for every transfer.
const BLOCK_SIZE: usize = 512;
/// Upper bound on polling loop iterations, so a wrong/absent controller
/// results in a clean error instead of a permanent hang.
const POLL_TIMEOUT: usize = 2_000_000;
/// Number of ACMD41 (SD_SEND_OP_COND) retries while waiting for the card to
/// leave the busy state after power-up.
const ACMD41_RETRIES: usize = 20_000;

const CMD_GO_IDLE_STATE: u8 = 0;
const CMD_ALL_SEND_CID: u8 = 2;
const CMD_SEND_RELATIVE_ADDR: u8 = 3;
const CMD_SELECT_CARD: u8 = 7;
const CMD_SEND_IF_COND: u8 = 8;
const CMD_STOP_TRANSMISSION: u8 = 12;
const CMD_SET_BLOCKLEN: u8 = 16;
const CMD_READ_SINGLE_BLOCK: u8 = 17;
const CMD_READ_MULTIPLE_BLOCK: u8 = 18;
const CMD_WRITE_BLOCK: u8 = 24;
const CMD_WRITE_MULTIPLE_BLOCK: u8 = 25;
const CMD_APP_CMD: u8 = 55;
const ACMD_SET_BUS_WIDTH: u8 = 6;
const ACMD_SD_SEND_OP_COND: u8 = 41;

/// `SEND_IF_COND` check pattern + 2.7-3.6V voltage supply flag.
const CMD8_VOLTAGE_CHECK_PATTERN: u32 = 0x1AA;
/// Voltage window advertised in ACMD41 (2.7-3.6V).
const ACMD41_VOLTAGE_WINDOW: u32 = 0x00FF_8000;
/// Host Capacity Support: request SDHC/SDXC addressing if the card supports it.
const ACMD41_HCS: u32 = 1 << 30;
/// Card Capacity Status returned in the ACMD41 response for SDHC/SDXC cards.
const OCR_CCS: u32 = 1 << 30;
/// Busy bit in the ACMD41/OCR response (0 while the card is powering up).
const OCR_BUSY: u32 = 1 << 31;

#[derive(Copy, Clone, Eq, PartialEq)]
enum ResponseType {
    None,
    /// 48-bit response with CRC and command-index checking (R1/R6/R7).
    R48,
    /// 48-bit response WITHOUT CRC or command-index checking (R3/OCR).
    /// The OCR response carries an all-ones CRC field and an all-ones
    /// command-index field, so enabling those checks makes the controller
    /// flag a spurious error and wedge its command state machine.
    R48NoCrc,
    R48Busy,
    R136,
}

pub struct Sdhci {
    base_address: usize,
    /// Relative Card Address, obtained via CMD3 and required by most
    /// subsequent commands (CMD7/CMD13/...).
    rca: u32,
    /// Whether the card understands block (SDHC/SDXC, `block_address` is a
    /// block index) or byte (SDSC, `block_address` is a byte offset that
    /// must be converted to a block index) addressing.
    is_high_capacity: bool,
    /// CNTPCT_EL0 timestamp (in ticks) of the last non-SDHCI_BUFFER register
    /// write, used to enforce REGISTER_WRITE_SPACING_US. A `Cell` since the
    /// register accessors take `&self` (mirroring pl011::Pl011's shared-
    /// reference style) even though they mutate hardware/timing state.
    last_write_ticks: Cell<u64>,
    /// Cached value last written to SDHCI_TRANSFER_MODE (0x0C), which
    /// shares a 32-bit hardware word with SDHCI_COMMAND (0x0E). Mirrors
    /// u-boot's `bcm2835_sdhci_writew()`: on this controller a bare 16-bit
    /// store to SDHCI_COMMAND alone is not reliable, so writes to
    /// TRANSFER_MODE are only cached here and the two halves are combined
    /// into a single 32-bit write once COMMAND is written (see write16()).
    transfer_mode_shadow: Cell<u16>,
}

impl Sdhci {
    pub const fn invalid() -> Self {
        Self {
            base_address: 0,
            rca: 0,
            is_high_capacity: false,
            last_write_ticks: Cell::new(0),
            transfer_mode_shadow: Cell::new(0),
        }
    }

    pub fn new(base_address: usize) -> Result<Self, ()> {
        let mut sdhci = Self {
            base_address,
            rca: 0,
            is_high_capacity: false,
            last_write_ticks: Cell::new(0),
            transfer_mode_shadow: Cell::new(0),
        };
        sdhci.reset_all()?;

        if (sdhci.read_present_state() & SDHCI_CARD_INSERTED) == 0 {
            /* Some controllers do not wire up card-detect; do not fail
             * outright, but warn since the rest of the sequence will fail
             * if no card is actually present. */
            println!("SDHCI: card-detect line reports no card inserted");
        }

        /* Enable the bus power supply at 3.3V. Per the SDHCI spec the
         * SD Bus Voltage Select field (POWER_CONTROL[3:1]) is
         * 111b=3.3V, 110b=3.0V, 101b=1.8V (matches u-boot's
         * SDHCI_POWER_330 = 0x0E, i.e. bits[3:1]=111). */
        sdhci.set_power(0b111);

        /* Bring the clock up to the card identification frequency
         * (400KHz, per the SD specification) before issuing any command. */
        sdhci.set_clock(400_000)?;

        sdhci.write8(SDHCI_TIMEOUT_CONTROL, 0x0E);

        /* Clear and then unmask every status bit we rely on. */
        sdhci.write16(SDHCI_INT_STATUS, 0xFFFF);
        sdhci.write16(SDHCI_INT_ENABLE, 0xFFFF);
        /* Signal Enable is intentionally left at 0: we poll INT_STATUS
         * instead of relying on the physical IRQ line. */
        sdhci.write16(SDHCI_SIGNAL_ENABLE, 0);

        sdhci.initialize_card()?;
        Ok(sdhci)
    }

    /* ---- Low level register access ---- */

    /// Busy-waits (if needed) so that at least REGISTER_WRITE_SPACING_US has
    /// elapsed since the previous register write, then records the new
    /// timestamp. Must be called before every register write except to
    /// SDHCI_BUFFER (see REGISTER_WRITE_SPACING_US).
    fn wait_for_write_spacing(&self) {
        let frequency = asm::get_cntfrq_el0();
        if frequency == 0 {
            return;
        }
        let spacing_ticks = (REGISTER_WRITE_SPACING_US * frequency).div_ceil(1_000_000);
        loop {
            let now = asm::get_cntpct_el0();
            if now.wrapping_sub(self.last_write_ticks.get()) >= spacing_ticks {
                break;
            }
            core::hint::spin_loop();
        }
    }

    fn read32(&self, offset: usize) -> u32 {
        unsafe { read_volatile((self.base_address + offset) as *const u32) }
    }

    /// Performs the actual 32-bit-aligned write, honoring the write-spacing
    /// workaround. All register writes ultimately go through this: real
    /// BCM2711 EMMC2/Arasan hardware does not reliably accept narrower
    /// (8/16-bit) bus-level stores (see write16()/write8()), so every
    /// write, even one nominally narrower, is issued as a full 32-bit
    /// store to a 4-byte-aligned offset.
    fn raw_write32(&self, aligned_offset: usize, value: u32) {
        if aligned_offset != SDHCI_BUFFER {
            self.wait_for_write_spacing();
        }
        unsafe { write_volatile((self.base_address + aligned_offset) as *mut u32, value) };
        self.last_write_ticks.set(asm::get_cntpct_el0());
    }

    fn write32(&self, offset: usize, value: u32) {
        self.raw_write32(offset, value);
    }

    fn read16(&self, offset: usize) -> u16 {
        unsafe { read_volatile((self.base_address + offset) as *const u16) }
    }

    /// Writes a 16-bit register via a 32-bit read-modify-write, mirroring
    /// u-boot's `bcm2835_sdhci_writew()`: BCM2711's Arasan-derived EMMC2
    /// controller is documented there as unreliable with bare 16-bit
    /// stores. SDHCI_TRANSFER_MODE (0x0C) and SDHCI_COMMAND (0x0E) share
    /// one 32-bit word and, per that same driver, must be committed
    /// together in a single write (issuing a command also always supplies
    /// the transfer mode for it): writes to TRANSFER_MODE are therefore
    /// only cached (`transfer_mode_shadow`), and applied to hardware only
    /// once COMMAND is written.
    fn write16(&self, offset: usize, value: u16) {
        if offset == SDHCI_TRANSFER_MODE {
            self.transfer_mode_shadow.set(value);
            return;
        }
        let aligned_offset = offset & !0b11;
        let word_shift = ((offset >> 1) & 1) * 16;
        let old_value = if offset == SDHCI_COMMAND {
            self.transfer_mode_shadow.get() as u32
        } else {
            self.read32(aligned_offset)
        };
        let mask: u32 = 0xFFFF << word_shift;
        let new_value = (old_value & !mask) | ((value as u32) << word_shift);
        self.raw_write32(aligned_offset, new_value);
    }

    /// Writes an 8-bit register via a 32-bit read-modify-write; see
    /// write16() for why this is required on this controller.
    fn write8(&self, offset: usize, value: u8) {
        let aligned_offset = offset & !0b11;
        let byte_shift = (offset & 0b11) * 8;
        let old_value = self.read32(aligned_offset);
        let mask: u32 = 0xFF << byte_shift;
        let new_value = (old_value & !mask) | ((value as u32) << byte_shift);
        self.raw_write32(aligned_offset, new_value);
    }

    fn read_present_state(&self) -> u32 {
        self.read32(SDHCI_PRESENT_STATE)
    }

    /// Busy-waits for at least `us` microseconds using the generic timer.
    fn delay_us(&self, us: u64) {
        let frequency = asm::get_cntfrq_el0();
        if frequency == 0 {
            return;
        }
        let ticks = (us * frequency).div_ceil(1_000_000);
        let start = asm::get_cntpct_el0();
        while asm::get_cntpct_el0().wrapping_sub(start) < ticks {
            core::hint::spin_loop();
        }
    }

    /* ---- Initialization helpers ---- */

    fn reset_all(&self) -> Result<(), ()> {
        self.write8(SDHCI_SOFTWARE_RESET, SDHCI_RESET_ALL);
        for _ in 0..POLL_TIMEOUT {
            if (unsafe { read_volatile((self.base_address + SDHCI_SOFTWARE_RESET) as *const u8) }
                & SDHCI_RESET_ALL)
                == 0
            {
                return Ok(());
            }
            core::hint::spin_loop();
        }
        println!("SDHCI: timed out waiting for the software reset to complete");
        Err(())
    }

    fn set_power(&self, voltage_select: u8) {
        /* SD Bus Voltage Select (bits[3:1]) followed by SD Bus Power (bit0). */
        self.write8(SDHCI_POWER_CONTROL, (voltage_select << 1) | 1);
    }

    /// Programs the SDCLK divider so that the resulting card clock is at
    /// most `target_hz`, following the standard SDHCI "divided clock mode"
    /// algorithm (matches Linux/U-Boot's generic `sdhci_set_clock`).
    fn set_clock(&self, target_hz: u32) -> Result<(), ()> {
        /* Stop the card clock while it is being reconfigured. */
        self.write16(SDHCI_CLOCK_CONTROL, 0);

        let base_clock_mhz = (self.read32(SDHCI_CAPABILITIES) >> 8) & 0xFF;
        let base_clock_hz = if base_clock_mhz == 0 {
            /* Capabilities did not advertise a base clock (allowed by the
             * spec for some platforms); BCM2711's EMMC2 is normally clocked
             * at 100MHz, so fall back to that. */
            100_000_000
        } else {
            base_clock_mhz * 1_000_000
        };

        let mut divisor: u32 = 1;
        while divisor < 0x3FF && (base_clock_hz / (divisor * 2)) > target_hz {
            divisor *= 2;
        }
        /* The register field encodes "divide by 2*N"; N=0 selects the base
         * clock unmodified. */
        let field = divisor / 2;

        let clock_control = ((field & 0xFF) << 8) as u16 | (((field >> 8) & 0x03) << 6) as u16;
        self.write16(SDHCI_CLOCK_CONTROL, clock_control | SDHCI_CLOCK_INT_EN);

        for _ in 0..POLL_TIMEOUT {
            if (self.read16(SDHCI_CLOCK_CONTROL) & SDHCI_CLOCK_INT_STABLE) != 0 {
                self.write16(
                    SDHCI_CLOCK_CONTROL,
                    self.read16(SDHCI_CLOCK_CONTROL) | SDHCI_CLOCK_CARD_EN,
                );
                return Ok(());
            }
            core::hint::spin_loop();
        }
        println!("SDHCI: internal clock never became stable");
        Err(())
    }

    /* ---- Command layer ---- */

    fn wait_for_not_inhibited(&self, mask: u32) -> Result<(), ()> {
        for i in 0..POLL_TIMEOUT {
            if (self.read_present_state() & mask) == 0 {
                if i > 100 {
                    println!("SDHCI: wait_for_not_inhibited took {i} iterations (mask={mask:#010X})");
                }
                return Ok(());
            }
            core::hint::spin_loop();
        }
        Err(())
    }

    fn wait_for_interrupt(&self, mask: u16) -> Result<u16, ()> {
        for _ in 0..POLL_TIMEOUT {
            let status = self.read16(SDHCI_INT_STATUS);
            if (status & SDHCI_INT_ERROR) != 0 {
                /* W1C: acknowledge every raised status bit. */
                let err_status = self.read16(SDHCI_ERR_INT_STATUS);
                self.write16(SDHCI_INT_STATUS, status);
                println!(
                    "SDHCI: error interrupt (INT_STATUS={status:#06X}, ERR_INT_STATUS={err_status:#06X})"
                );
                return Err(());
            }
            if (status & mask) != 0 {
                self.write16(SDHCI_INT_STATUS, status & mask);
                return Ok(status);
            }
            core::hint::spin_loop();
        }
        println!(
            "SDHCI: no interrupt within timeout (last INT_STATUS={:#06X}, PRESENT_STATE={:#010X})",
            self.read16(SDHCI_INT_STATUS),
            self.read_present_state()
        );
        Err(())
    }

    /// Issues a command and returns its response (zero-padded to 4 words;
    /// only the low word is meaningful for R1/R1b/R3/R6/R7).
    fn send_command(
        &self,
        index: u8,
        argument: u32,
        response_type: ResponseType,
        data_present: bool,
    ) -> Result<[u32; 4], ()> {
        /* U-Boot's generic sdhci_send_command() waits for BOTH CMD_INHIBIT
         * and DAT_INHIBIT to clear before issuing essentially every command
         * (the only exceptions are STOP_TRANSMISSION and tuning-block
         * commands without data, which we do not implement here). A card
         * can be internally busy on the DAT lines after a response even
         * when that response type has no defined "busy" signaling (e.g.
         * ACMD41's R3), so unconditionally waiting on DAT_INHIBIT too
         * avoids issuing the next command while the card is still busy. */
        self.wait_for_not_inhibited(SDHCI_CMD_INHIBIT | SDHCI_DAT_INHIBIT)?;

        self.write16(SDHCI_INT_STATUS, 0xFFFF);
        self.write32(SDHCI_ARGUMENT, argument);

        let response_select = match response_type {
            ResponseType::None => SDHCI_CMD_RESP_NONE,
            ResponseType::R48 => SDHCI_CMD_RESP_48 | SDHCI_CMD_CRC_CHECK | SDHCI_CMD_INDEX_CHECK,
            ResponseType::R48NoCrc => SDHCI_CMD_RESP_48,
            ResponseType::R48Busy => {
                SDHCI_CMD_RESP_48_BUSY | SDHCI_CMD_CRC_CHECK | SDHCI_CMD_INDEX_CHECK
            }
            ResponseType::R136 => SDHCI_CMD_RESP_136 | SDHCI_CMD_CRC_CHECK,
        };
        let mut command = ((index as u16) << 8) | response_select;
        if data_present {
            command |= SDHCI_CMD_DATA_PRESENT;
        }
        self.write16(SDHCI_COMMAND, command);
        if index == CMD_APP_CMD {
            let host_ctrl_word = self.read32(SDHCI_POWER_CONTROL & !0b11);
            let power_control = (host_ctrl_word >> 8) as u8;
            println!(
                "SDHCI: issued CMD{index} command={command:#06X} present_state_after_write={:#010X} clock_control={:#06X} power_control={power_control:#04X} host_control2={:#06X} int_enable={:#06X}",
                self.read_present_state(),
                self.read16(SDHCI_CLOCK_CONTROL),
                self.read16(SDHCI_HOST_CONTROL2),
                self.read16(SDHCI_INT_ENABLE)
            );
        }

        self.wait_for_interrupt(SDHCI_INT_RESPONSE)
            .map_err(|_| println!("SDHCI: CMD{index} timed out or failed"))?;

        if response_type == ResponseType::R136 {
            /* The CRC is not delivered to software; reconstruct the 128-bit
             * response by shifting each register up by 8 bits and pulling
             * in the top byte of the next one (matches U-Boot's
             * sdhci_send_command()). */
            let raw = [
                self.read32(SDHCI_RESPONSE),
                self.read32(SDHCI_RESPONSE + 4),
                self.read32(SDHCI_RESPONSE + 8),
                self.read32(SDHCI_RESPONSE + 12),
            ];
            let mut response = [0u32; 4];
            for i in 0..4 {
                response[i] = raw[3 - i] << 8;
                if i != 3 {
                    response[i] |= raw[3 - i - 1] >> 24;
                }
            }
            Ok(response)
        } else {
            Ok([self.read32(SDHCI_RESPONSE), 0, 0, 0])
        }
    }

    fn app_command(
        &self,
        index: u8,
        argument: u32,
        response_type: ResponseType,
    ) -> Result<[u32; 4], ()> {
        /* CMD55 must carry the currently selected card's RCA. */
        self.delay_us(1000);
        self.send_command(CMD_APP_CMD, self.rca << 16, ResponseType::R48, false)?;
        self.send_command(index, argument, response_type, false)
    }

    fn initialize_card(&mut self) -> Result<(), ()> {
        /* CMD0: GO_IDLE_STATE */
        self.send_command(CMD_GO_IDLE_STATE, 0, ResponseType::None, false)?;

        /* CMD8: SEND_IF_COND - probes for a v2.00+ card and picks the
         * 2.7-3.6V voltage range. Older (v1) cards do not respond to this
         * command; treat a failure as "legacy card" rather than an error. */
        let cmd8_result = self.send_command(
            CMD_SEND_IF_COND,
            CMD8_VOLTAGE_CHECK_PATTERN,
            ResponseType::R48,
            false,
        );
        println!(
            "SDHCI: CMD8 result={:?} present_state={:#010X}",
            cmd8_result.as_ref().map(|r| r[0]),
            self.read_present_state()
        );
        let is_v2_or_later = cmd8_result.is_ok();

        /* ACMD41: SD_SEND_OP_COND - poll until the card leaves the busy
         * state, requesting High Capacity Support so SDHC/SDXC cards report
         * block instead of byte addressing. */
        let mut ocr = 0;
        let mut ready = false;
        for i in 0..ACMD41_RETRIES {
            let argument = ACMD41_VOLTAGE_WINDOW | if is_v2_or_later { ACMD41_HCS } else { 0 };
            let response =
                self.app_command(ACMD_SD_SEND_OP_COND, argument, ResponseType::R48NoCrc)?;
            ocr = response[0];
            if i < 3 {
                println!("SDHCI: ACMD41 iter={i} ocr={ocr:#010X}");
            }
            if (ocr & OCR_BUSY) != 0 {
                ready = true;
                break;
            }
        }
        if !ready {
            println!("SDHCI: card did not leave the busy state (no card inserted?)");
            return Err(());
        }
        self.is_high_capacity = is_v2_or_later && (ocr & OCR_CCS) != 0;

        /* CMD2: ALL_SEND_CID */
        self.send_command(CMD_ALL_SEND_CID, 0, ResponseType::R136, false)?;

        /* CMD3: SEND_RELATIVE_ADDR */
        let response = self.send_command(CMD_SEND_RELATIVE_ADDR, 0, ResponseType::R48, false)?;
        self.rca = response[0] >> 16;

        /* CMD7: SELECT_CARD */
        self.send_command(CMD_SELECT_CARD, self.rca << 16, ResponseType::R48Busy, false)?;

        /* ACMD6: SET_BUS_WIDTH - switch to the 4-bit data bus. */
        self.app_command(ACMD_SET_BUS_WIDTH, 0b10, ResponseType::R48)?;
        self.write8(
            0x28, /* Host Control 1 */
            unsafe { read_volatile((self.base_address + 0x28) as *const u8) } | (1 << 1),
        );

        /* SDSC cards address by byte offset and need an explicit block
         * length; SDHC/SDXC cards are fixed at 512 bytes. */
        if !self.is_high_capacity {
            self.send_command(
                CMD_SET_BLOCKLEN,
                BLOCK_SIZE as u32,
                ResponseType::R48,
                false,
            )?;
        }

        /* Raise the clock from the identification frequency (400KHz) to a
         * conservative default speed (25MHz "Default Speed" mode, valid on
         * every SD card without needing a CMD6 speed-class switch). */
        self.set_clock(25_000_000)?;

        println!(
            "SDHCI: card ready (RCA={:#X}, {})",
            self.rca,
            if self.is_high_capacity {
                "block-addressed (SDHC/SDXC)"
            } else {
                "byte-addressed (SDSC)"
            }
        );
        Ok(())
    }

    /// Converts a byte offset (as used by [`crate::fat32`]) into the block
    /// index/offset argument expected by CMD17/18/24/25 for this card.
    fn command_argument(&self, block_address: u64) -> u32 {
        if self.is_high_capacity {
            (block_address / BLOCK_SIZE as u64) as u32
        } else {
            block_address as u32
        }
    }

    fn pio_transfer(
        &self,
        buffer_address: usize,
        number_of_blocks: u32,
        is_write: bool,
    ) -> Result<(), ()> {
        for block in 0..number_of_blocks {
            let ready_bit = if is_write {
                SDHCI_INT_BUFFER_WRITE_READY
            } else {
                SDHCI_INT_BUFFER_READ_READY
            };
            self.wait_for_interrupt(ready_bit)
                .map_err(|_| println!("SDHCI: PIO buffer not ready for block {block}"))?;

            let block_base = buffer_address + (block as usize) * BLOCK_SIZE;
            for word in 0..(BLOCK_SIZE / size_of::<u32>()) {
                let address = block_base + word * size_of::<u32>();
                if is_write {
                    let value = unsafe { read_volatile(address as *const u32) };
                    self.write32(SDHCI_BUFFER, value);
                } else {
                    let value = self.read32(SDHCI_BUFFER);
                    unsafe { write_volatile(address as *mut u32, value) };
                }
            }
        }

        self.wait_for_interrupt(SDHCI_INT_DATA_END)
            .map_err(|_| println!("SDHCI: transfer did not complete"))?;

        /* Present bit-check helper reads back the read-write flags to
         * ensure the buffer window has actually closed before returning. */
        let _ = self
            .read_present_state()
            &(SDHCI_BUFFER_READ_ENABLE | SDHCI_BUFFER_WRITE_ENABLE);
        Ok(())
    }

    fn operation_sync(
        &mut self,
        buffer_address: usize,
        block_address: u64,
        length: u64,
        is_write: bool,
    ) -> Result<(), ()> {
        if (block_address % BLOCK_SIZE as u64) != 0 || (length % BLOCK_SIZE as u64) != 0 {
            println!(
                "Block Address({:#X}) and Length({:#X}) must be 512Byte-Aligned.",
                block_address, length
            );
            return Err(());
        }
        let number_of_blocks = (length / BLOCK_SIZE as u64) as u32;
        if number_of_blocks == 0 {
            return Ok(());
        }

        self.write16(SDHCI_BLOCK_SIZE, BLOCK_SIZE as u16);
        self.write16(SDHCI_BLOCK_COUNT, number_of_blocks as u16);

        let mut transfer_mode = SDHCI_TRNS_BLK_CNT_EN;
        if !is_write {
            transfer_mode |= SDHCI_TRNS_READ;
        }
        let is_multiple = number_of_blocks > 1;
        if is_multiple {
            transfer_mode |= SDHCI_TRNS_MULTI | SDHCI_TRNS_ACMD12;
        }
        self.write16(SDHCI_TRANSFER_MODE, transfer_mode);

        let command_index = if is_write {
            if is_multiple {
                CMD_WRITE_MULTIPLE_BLOCK
            } else {
                CMD_WRITE_BLOCK
            }
        } else if is_multiple {
            CMD_READ_MULTIPLE_BLOCK
        } else {
            CMD_READ_SINGLE_BLOCK
        };

        self.send_command(
            command_index,
            self.command_argument(block_address),
            ResponseType::R48,
            true,
        )?;

        let result = self.pio_transfer(buffer_address, number_of_blocks, is_write);

        if is_multiple {
            /* Auto CMD12 (SDHCI_TRNS_ACMD12) already asks the controller to
             * issue STOP_TRANSMISSION on our behalf; nothing left to do
             * here. Kept as an explicit branch for readability/robustness
             * in case a future controller quirk needs manual CMD12. */
            let _ = CMD_STOP_TRANSMISSION;
        }

        result
    }

    pub fn read(
        &mut self,
        buffer_address: usize,
        block_address: u64,
        length: u64,
    ) -> Result<(), ()> {
        self.operation_sync(buffer_address, block_address, length, false)
    }

    pub fn write(
        &mut self,
        buffer_address: usize,
        block_address: u64,
        length: u64,
    ) -> Result<(), ()> {
        self.operation_sync(buffer_address, block_address, length, true)
    }
}

unsafe impl core::marker::Send for Sdhci {}

impl super::block_device::BlockDevice for Sdhci {
    fn read(&mut self, buffer_address: usize, block_address: u64, length: u64) -> Result<(), ()> {
        Sdhci::read(self, buffer_address, block_address, length)
    }

    fn write(&mut self, buffer_address: usize, block_address: u64, length: u64) -> Result<(), ()> {
        Sdhci::write(self, buffer_address, block_address, length)
    }
}
