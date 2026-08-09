//!
//! PL011 MMIO Driver
//!

use crate::mmio::gicv2::GicDistributorMmio;
use crate::vm::MmioHandler;

const UART_DR: usize = 0x000;
const UART_RSR_ECR: usize = 0x004;
const UART_FR: usize = 0x018;
const UART_ILPR: usize = 0x020;
const UART_IBRD: usize = 0x024;
const UART_FBRD: usize = 0x028;
const UART_LCR_H: usize = 0x02C;
const UART_CR: usize = 0x030;
const UART_IFLS: usize = 0x034;
const UART_IMSC: usize = 0x038;
const UART_RIS: usize = 0x03C;
const UART_MIS: usize = 0x040;
const UART_ICR: usize = 0x044;
const UART_DMACR: usize = 0x048;
const UART_PERIPH_ID0: usize = 0xFE0;
const UART_PERIPH_ID1: usize = 0xFE4;
const UART_PERIPH_ID2: usize = 0xFE8;
const UART_PERIPH_ID3: usize = 0xFEC;
const UART_PCELL_ID0: usize = 0xFF0;
const UART_PCELL_ID1: usize = 0xFF4;
const UART_PCELL_ID2: usize = 0xFF8;
const UART_PCELL_ID3: usize = 0xFFC;

/// Bit indicating whether the RX FIFO is empty
const UART_FR_RXFE: u16 = 1 << 4;
/// Bit indicating whether the RX FIFO is full
const UART_FR_RXFF: u16 = 1 << 6;
/// Bit indicating whether the receive interrupt is enabled
const UART_IMSC_RXIM: u16 = 1 << 4;
/// Bit indicating whether a receive interrupt has occurred
const UART_RIS_RXRIS: u16 = 1 << 4;
/// PL011's virtual interrupt number
const PL011_INT_ID: u32 = 33;

/// Depth of the emulated RX FIFO. A real PL011 has a 16-byte FIFO, but the
/// old 4-byte buffer silently dropped bytes whenever the guest did not drain
/// it quickly enough (e.g. pasted text or any fast input burst). Since this
/// is an emulation with no real baud-rate limit, a much deeper FIFO is cheap
/// and makes fast host-side input reliable.
const RX_FIFO_SIZE: usize = 256;

pub struct Pl011Mmio {
    flag: u16,
    interrupt_mask: u16,
    raw_interrupt_status: u16,
    control: u16,
    /* RX FIFO as a ring buffer: `rx_fifo_head` indexes the oldest buffered
     * byte and `rx_fifo_len` counts buffered bytes. An explicit length (rather
     * than the old "0 means empty slot" scheme) also allows NUL bytes to be
     * received like any other byte. */
    rx_fifo: [u8; RX_FIFO_SIZE],
    rx_fifo_head: usize,
    rx_fifo_len: usize,
    /* The following registers have no effect on this emulation's actual
     * behavior (there is no real baud-rate generator, FIFO depth, or
     * DMA engine to configure, and this virtual UART never reports a
     * framing/parity/break/overrun receive error), but Linux's amba-pl011
     * driver reads back several of them during probe/`set_termios` (e.g.
     * to verify IBRD/FBRD took effect) and DMA engine drivers probe
     * UART011_DMACR. Storing and returning whatever the guest last wrote
     * avoids that stale reads of `0x00` are mistaken for "value rejected"
     * and avoids the guest's own `dev_err`/retry logic around them. */
    irda_low_power_counter: u16,
    integer_baud_rate_divisor: u16,
    fractional_baud_rate_divisor: u16,
    line_control: u16,
    interrupt_fifo_level_select: u16,
    dma_control: u16,
}

impl Pl011Mmio {
    pub fn new() -> Self {
        Self {
            /* The RX FIFO starts empty, so RXFE must be set from the
             * beginning (the old code left it clear until the first read,
             * making the guest believe a byte was already waiting). */
            flag: UART_FR_RXFE,
            interrupt_mask: 0,
            raw_interrupt_status: 0,
            control: 0,
            rx_fifo: [0; RX_FIFO_SIZE],
            rx_fifo_head: 0,
            rx_fifo_len: 0,
            irda_low_power_counter: 0,
            integer_baud_rate_divisor: 0,
            fractional_baud_rate_divisor: 0,
            line_control: 0,
            interrupt_fifo_level_select: 0,
            dma_control: 0,
        }
    }

    pub fn push(&mut self, data: u8, distributor: &mut GicDistributorMmio) {
        if self.rx_fifo_len == RX_FIFO_SIZE {
            /* FIFO full: drop the incoming byte (same policy as before, but
             * now only reachable after 256 unread bytes instead of 4). */
            return;
        }
        let tail = (self.rx_fifo_head + self.rx_fifo_len) % RX_FIFO_SIZE;
        self.rx_fifo[tail] = data;
        self.rx_fifo_len += 1;
        self.flag &= !UART_FR_RXFE;
        if self.rx_fifo_len == RX_FIFO_SIZE {
            self.flag |= UART_FR_RXFF;
        }
        if (self.interrupt_mask & UART_IMSC_RXIM) != 0 {
            self.raw_interrupt_status |= UART_RIS_RXRIS;
            distributor.trigger_interrupt(PL011_INT_ID, None);
        }
    }
}

impl MmioHandler for Pl011Mmio {
    fn read(&mut self, offset: usize, _access_width: u64) -> Result<u64, ()> {
        let value: u64;
        match offset {
            UART_DR => {
                if self.rx_fifo_len == 0 {
                    value = 0;
                } else {
                    value = self.rx_fifo[self.rx_fifo_head] as u64;
                    self.rx_fifo_head = (self.rx_fifo_head + 1) % RX_FIFO_SIZE;
                    self.rx_fifo_len -= 1;
                    self.flag &= !UART_FR_RXFF;
                    if self.rx_fifo_len == 0 {
                        self.flag |= UART_FR_RXFE;
                        self.raw_interrupt_status &= !(UART_RIS_RXRIS);
                    }
                }
            }
            UART_FR => {
                value = self.flag as u64;
            }
            UART_CR => {
                value = self.control as u64;
            }
            UART_IMSC => {
                value = self.interrupt_mask as u64;
            }
            UART_RIS => {
                value = self.raw_interrupt_status as u64;
            }
            UART_MIS => {
                /* Masked Interrupt Status: which raw interrupt sources are both
                 * pending (RIS) and unmasked (IMSC). Linux's amba-pl011 IRQ
                 * handler reads this (not RIS) to decide which sources to
                 * service, so leaving it hardwired to 0 would make the guest
                 * believe no interrupt was ever actually pending. */
                value = (self.raw_interrupt_status & self.interrupt_mask) as u64;
            }
            UART_RSR_ECR => {
                /* This virtual UART never produces a framing/parity/break/
                 * overrun receive error, so there is nothing to report here. */
                value = 0x00;
            }
            UART_ILPR => {
                value = self.irda_low_power_counter as u64;
            }
            UART_IBRD => {
                value = self.integer_baud_rate_divisor as u64;
            }
            UART_FBRD => {
                value = self.fractional_baud_rate_divisor as u64;
            }
            UART_LCR_H => {
                value = self.line_control as u64;
            }
            UART_IFLS => {
                value = self.interrupt_fifo_level_select as u64;
            }
            UART_DMACR => {
                value = self.dma_control as u64;
            }
            UART_PERIPH_ID0 => {
                value = 0x11;
            }
            UART_PERIPH_ID1 => {
                value = 0x01 << 4;
            }
            UART_PERIPH_ID2 => {
                value = (0x03 << 4) | 0x04;
            }
            UART_PERIPH_ID3 => {
                value = 0x00;
            }
            UART_PCELL_ID0 => {
                value = 0x0D;
            }
            UART_PCELL_ID1 => {
                value = 0xF0;
            }
            UART_PCELL_ID2 => {
                value = 0x05;
            }
            UART_PCELL_ID3 => {
                value = 0xB1;
            }
            _ => {
                value = 0x00; /* unimplemented */
            }
        }
        Ok(value)
    }

    fn write(&mut self, offset: usize, _access_width: u64, value: u64) -> Result<(), ()> {
        match offset {
            UART_DR => {
                print!("{}", value as u8 as char);
            }
            UART_CR => {
                self.control = value as u16;
            }
            UART_IMSC => {
                self.interrupt_mask = value as u16;
            }
            UART_ICR => {
                self.raw_interrupt_status &= !(value as u16);
            }
            UART_RSR_ECR => {
                /* Write-to-clear for the (always-empty) receive error state;
                 * nothing to actually clear, but accepting the write instead
                 * of falling into the unimplemented default avoids surprising
                 * a guest driver that writes here unconditionally on open. */
            }
            UART_ILPR => {
                self.irda_low_power_counter = value as u16;
            }
            UART_IBRD => {
                self.integer_baud_rate_divisor = value as u16;
            }
            UART_FBRD => {
                self.fractional_baud_rate_divisor = value as u16;
            }
            UART_LCR_H => {
                self.line_control = value as u16;
            }
            UART_IFLS => {
                self.interrupt_fifo_level_select = value as u16;
            }
            UART_DMACR => {
                self.dma_control = value as u16;
            }
            _ => { /* unimplemented */ }
        }
        Ok(())
    }
}
