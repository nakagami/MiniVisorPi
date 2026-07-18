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
const UART_CR: usize = 0x030;
const UART_IMSC: usize = 0x038;
/// Minimum MMIO range this driver actually touches: the highest register it
/// accesses is UART_IMSC, read/written as a u16, so this is UART_IMSC + 2.
/// This must NOT simply be the PL011's whole 4 KiB page size (as QEMU's
/// `virt` machine reports for its pl011@9000000 node): the Raspberry Pi 4's
/// official devicetree instead gives its PL011 a much tighter "reg" range
/// (0x200 bytes), which is still ample for every register this driver uses,
/// so requiring a full page here would wrongly reject valid real hardware.
const UART_SIZE: usize = UART_IMSC + 2;

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
            ptr::write_volatile(
                (self.base_address + UART_CR) as *mut u16,
                UART_CR_RXE | UART_CR_TXE | UART_CR_UARTEN,
            );
            ptr::write_volatile(
                (self.base_address + UART_IMSC) as *mut u16,
                ptr::read_volatile((self.base_address + UART_IMSC) as *const u16) | UART_IMSC_RXIM,
            );
        }
    }
}

/// Implementation required for use with the Serial struct
impl serial::SerialDevice for Pl011 {
    fn putc(&self, c: u8) -> Result<(), Error> {
        while self.is_tx_fifo_full() {
            core::hint::spin_loop();
        }
        unsafe { ptr::write_volatile((self.base_address + UART_DR) as *mut u8, c) };
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
