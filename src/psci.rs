//!
//!  Power State Coordination Interface
//!

use crate::asm::smc;

use core::sync::atomic::{AtomicU64, Ordering};

const PSCI_VERSION: u64 = 0x8400_0000;
const PSCI_SYSTEM_OFF: u64 = 0x8400_0008;
const PSCI_CPU_ON: u64 = 0xC400_0003;

/// Stack pointer to hand to a CPU woken up via the ARM "spin-table" boot
/// protocol (see [`crate::asm::spin_table_entry`]). Must be written just
/// before [`spin_table_cpu_on`] arms the release address, and is consumed
/// exactly once by the waking core.
pub static SPIN_TABLE_SP: AtomicU64 = AtomicU64::new(0);


#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum PsciErrorCodes {
    Success,
    NotSupported,
    InvalidParameters,
    Denied,
    AlreadyOn,
    OnPending,
    InternalFailure,
    NotPresent,
    Disabled,
    InvalidAddress,
    Unknown,
}

impl From<u64> for PsciErrorCodes {
    fn from(value: u64) -> Self {
        let value = value as i32;
        match value {
            0 => Self::Success,
            -1 => Self::NotSupported,
            -2 => Self::InvalidParameters,
            -3 => Self::Denied,
            -4 => Self::AlreadyOn,
            -5 => Self::OnPending,
            -6 => Self::InternalFailure,
            -7 => Self::NotPresent,
            -8 => Self::Disabled,
            -9 => Self::InvalidAddress,
            _ => Self::Unknown,
        }
    }
}

pub fn check_psci_version() -> Result<(u16, u16), PsciErrorCodes> {
    let version = unsafe { smc(PSCI_VERSION, 0, 0, 0) };
    if version as i32 == -1 {
        return Err(PsciErrorCodes::NotSupported);
    }
    let major_version = (version >> 16) as u16;
    let minor_version = (version & (u16::MAX as u64)) as u16;
    Ok((major_version, minor_version))
}

pub fn cpu_on(target_cpu: u64, entry_point: u64, argument: u64) -> Result<(), PsciErrorCodes> {
    let result = unsafe { smc(PSCI_CPU_ON, target_cpu, entry_point, argument) };
    let error_code = PsciErrorCodes::from(result);
    if error_code == PsciErrorCodes::Success {
        Ok(())
    } else {
        Err(error_code)
    }
}

pub fn system_off() -> ! {
    unsafe { smc(PSCI_SYSTEM_OFF, 0, 0, 0) };
    unreachable!()
}

/// Brings up a secondary CPU using the ARM "spin-table" boot protocol,
/// used by platforms without PSCI firmware (e.g. Raspberry Pi 4's stock
/// firmware, whose `cpu` DTB nodes advertise
/// `enable-method = "spin-table"` and a per-core `cpu-release-addr`).
///
/// `release_address` is the physical address read from the target CPU's
/// `cpu-release-addr` DTB property: the platform firmware parks the core
/// in a `wfe` loop polling that address, and jumps to whatever entry point
/// is written there once woken with `sev`.
pub fn spin_table_cpu_on(release_address: usize, stack_pointer: u64) {
    SPIN_TABLE_SP.store(stack_pointer, Ordering::SeqCst);
    let entry_point = crate::asm::spin_table_entry as *const () as u64;
    unsafe {
        core::ptr::write_volatile(release_address as *mut u64, entry_point);
        crate::asm::dsb_sy();
        crate::asm::sev();
    }
}

