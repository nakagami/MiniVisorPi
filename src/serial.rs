//!
//! Module for character output
//!
use crate::lock::Mutex;

use core::fmt;
use core::marker::Send;

/// Functions that a serial device struct must implement
/// (for abstraction purposes)
pub trait SerialDevice {
    fn putc(&self, c: u8) -> Result<(), fmt::Error>;
    fn getc(&self) -> Result<Option<u8>, fmt::Error>;
}

pub struct Serial<'a> {
    inner: Option<&'a Mutex<dyn SerialDevice + Send>>,
}

/// Function written to by print!/println!
static SERIAL_DEVICE: Mutex<Serial> = Mutex::new(Serial { inner: None });

impl<'a> Serial<'a> {
    pub fn new(device: &'a Mutex<dyn SerialDevice + Send>) -> Self {
        Self {
            inner: Some(device),
        }
    }
}

/// Implementation required for using write_fmt
impl fmt::Write for Serial<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let Some(inner) = self.inner else {
            return Err(fmt::Error {});
        };
        let inner = inner.lock();
        for c in s.as_bytes() {
            if *c == b'\n' {
                inner.putc(b'\r')?;
            }
            inner.putc(*c)?;
        }
        Ok(())
    }
}

pub fn init_default_serial_port(device: &'static Mutex<dyn SerialDevice + Send>) {
    *SERIAL_DEVICE.lock() = Serial::new(device);
}

/// Function called from print!/println!
pub fn print(args: fmt::Arguments) {
    use fmt::Write;
    let _ = SERIAL_DEVICE.lock().write_fmt(args);
}

#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {
        $crate::serial::print(format_args!($($arg)*))
    };
}

#[macro_export]
macro_rules! println {
    ()=>( ($crate::serial::print(format_args!("\n"))));
    ($fmt:expr) => ($crate::serial::print(format_args!("{}\n", format_args!($fmt))));
    ($fmt:expr, $($arg:tt)*) => ($crate::serial::print(format_args!("{}\n", format_args!($fmt, $($arg)*))));
}
