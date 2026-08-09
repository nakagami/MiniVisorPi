//!
//! Arm PL011 device driver
//!
use crate::serial;

use core::fmt::Error;
use core::ptr;

pub struct Pl011 {
    base_address: usize,
}

const UART_DR: usize = 0x000;
const UART_FR: usize = 0x018;
const UART_IFLS: usize = 0x034;
const UART_CR: usize = 0x030;
const UART_IMSC: usize = 0x038;
const UART_ICR: usize = 0x044;
/// Minimum MMIO range this driver actually touches: the highest register it
/// accesses is UART_ICR, read/written as a u16, so this is UART_ICR + 2.
/// This must NOT simply be the PL011's whole 4 KiB page size (as QEMU's
/// `virt` machine reports for its pl011@9000000 node): the Raspberry Pi 4's
/// official devicetree instead gives its PL011 a much tighter "reg" range
/// (0x200 bytes), which is still ample for every register this driver uses,
/// so requiring a full page here would wrongly reject valid real hardware.
const UART_SIZE: usize = UART_ICR + 2;

/// Bit indicating whether the TX FIFO is full
const UART_FR_TXFF: u16 = 1 << 5;
/// Bit indicating whether the RX FIFO is empty
const UART_FR_RXFE: u16 = 1 << 4;
/// Bit indicating whether reception is enabled
const UART_CR_RXE: u16 = 1 << 9;
/// Bit indicating whether transmission is enabled
const UART_CR_TXE: u16 = 1 << 8;
/// Bit indicating whether the UART is enabled
const UART_CR_UARTEN: u16 = 1;
/// Bit indicating whether the receive interrupt is enabled
const UART_IMSC_RXIM: u16 = 1 << 4;
/// Bit indicating whether the receive *timeout* interrupt is enabled.
/// This one is essential for an interactive console: the boot firmware (U-Boot's
/// pl01x driver writes `LCRH = WLEN_8 | FEN`) leaves the FIFOs enabled, and with
/// FEN=1 the plain receive interrupt (RXIM) only asserts once the RX FIFO has
/// filled to the UARTIFLS trigger level. A single keystroke therefore never
/// raises RXRIS on its own -- only the receive timeout, which fires after the
/// RX line has been idle for 32 bit periods with unread data in the FIFO, does.
const UART_IMSC_RTIM: u16 = 1 << 6;
/// Field selecting the RX FIFO level at which RXRIS asserts.
const UART_IFLS_RXIFLSEL: u16 = 0b111 << 3;
/// RX FIFO trigger level of 1/8 full, i.e. assert RXRIS as soon as 4 of the 32
/// entries are occupied, rather than the reset value of 1/2 (16 entries).
const UART_IFLS_RXIFLSEL_1_8: u16 = 0b000 << 3;
/// Bit clearing a pending receive interrupt
const UART_ICR_RXIC: u16 = 1 << 4;
/// Bit clearing a pending receive timeout interrupt
const UART_ICR_RTIC: u16 = 1 << 6;

impl Pl011 {
    pub const fn invalid() -> Self {
        Self { base_address: 0 }
    }

    pub fn new(base_address: usize, range: usize) -> Result<Self, ()> {
        if range < UART_SIZE {
            return Err(());
        }
        Ok(Self { base_address })
    }

    fn is_tx_fifo_full(&self) -> bool {
        (unsafe { ptr::read_volatile((self.base_address + UART_FR) as *const u16) } & UART_FR_TXFF)
            != 0
    }

    fn is_rx_fifo_empty(&self) -> bool {
        (unsafe { ptr::read_volatile((self.base_address + UART_FR) as *const u16) } & UART_FR_RXFE)
            != 0
    }

    pub fn enable_interrupt(&self) {
        unsafe {
            /* Read-modify-write instead of overwriting: the boot firmware may have
             * configured further bits here (e.g. hardware flow control) that this
             * driver has no business clearing. */
            ptr::write_volatile(
                (self.base_address + UART_CR) as *mut u16,
                ptr::read_volatile((self.base_address + UART_CR) as *const u16)
                    | UART_CR_RXE
                    | UART_CR_TXE
                    | UART_CR_UARTEN,
            );
            /* Lower the RX FIFO trigger level so that RXRIS asserts after a few
             * characters rather than only at half-full. The receive timeout below
             * still covers the single-keystroke case. */
            ptr::write_volatile(
                (self.base_address + UART_IFLS) as *mut u16,
                (ptr::read_volatile((self.base_address + UART_IFLS) as *const u16)
                    & !UART_IFLS_RXIFLSEL)
                    | UART_IFLS_RXIFLSEL_1_8,
            );
            /* Discard anything that went pending before the handler was in place. */
            self.clear_rx_interrupt();
            ptr::write_volatile(
                (self.base_address + UART_IMSC) as *mut u16,
                ptr::read_volatile((self.base_address + UART_IMSC) as *const u16)
                    | UART_IMSC_RXIM
                    | UART_IMSC_RTIM,
            );
        }
    }

    /// Acknowledges the RX/RX-timeout interrupt at the UART itself. The PL011's
    /// output is wired to a level-triggered SPI, so without this write the line
    /// stays asserted and the GIC re-signals the very same interrupt forever:
    /// draining UART_DR alone clears only RXRIS, never RTRIS.
    pub fn clear_rx_interrupt(&self) {
        unsafe {
            ptr::write_volatile(
                (self.base_address + UART_ICR) as *mut u16,
                UART_ICR_RXIC | UART_ICR_RTIC,
            )
        };
    }
}

/// Diagnostic counters for physical console output cost (see the `stat`
/// console command). Cycle values are in CNTPCT_EL0 ticks.
pub static PHYS_PUTC_COUNT: core::sync::atomic::AtomicU64 =
    core::sync::atomic::AtomicU64::new(0);
pub static PHYS_PUTC_CYCLES: core::sync::atomic::AtomicU64 =
    core::sync::atomic::AtomicU64::new(0);

/// Implementation required for use with the Serial struct
impl serial::SerialDevice for Pl011 {
    fn putc(&self, c: u8) -> Result<(), Error> {
        let stat_start = crate::asm::get_cntpct_el0();
        while self.is_tx_fifo_full() {
            core::hint::spin_loop();
        }
        unsafe { ptr::write_volatile((self.base_address + UART_DR) as *mut u8, c) };
        use core::sync::atomic::Ordering;
        PHYS_PUTC_COUNT.fetch_add(1, Ordering::Relaxed);
        PHYS_PUTC_CYCLES.fetch_add(crate::asm::get_cntpct_el0() - stat_start, Ordering::Relaxed);
        Ok(())
    }

    fn getc(&self) -> Result<Option<u8>, Error> {
        if self.is_rx_fifo_empty() {
            return Ok(None);
        }
        Ok(Some(unsafe {
            ptr::read_volatile((self.base_address + UART_DR) as *const u8)
        }))
    }
}
