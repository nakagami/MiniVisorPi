//!
//! Assembly module
//!

use core::arch::{asm, naked_asm};

pub fn get_currentel() -> u64 {
    let currentel: u64;
    unsafe { asm!("mrs {}, currentel", out(reg) currentel) };
    currentel
}

pub unsafe fn set_hcr_el2(hcr_el2: u64) {
    unsafe { asm!("msr hcr_el2, {}", in(reg) hcr_el2) };
}

pub unsafe fn set_elr_el2(elr_el2: u64) {
    unsafe { asm!("msr elr_el2, {}", in(reg) elr_el2) };
}

pub unsafe fn set_spsr_el2(spsr_el2: u64) {
    unsafe { asm!("msr spsr_el2, {}", in(reg) spsr_el2) };
}

pub unsafe fn eret(x0: u64, x1: u64, x2: u64, x3: u64) -> ! {
    unsafe {
        asm!("eret",
             in("x0") x0,
             in("x1") x1,
             in("x2") x2,
             in("x3") x3,
             options(noreturn))
    }
}

pub fn get_stack_pointer() -> u64 {
    let sp: u64;
    unsafe { asm!("mov {}, sp", out(reg) sp) };
    sp
}

/// Reads the EL2 physical generic timer's free-running counter
/// (`CNTPCT_EL0`), ticking at `get_cntfrq_el0()` Hz. Used for short
/// microsecond-scale busy-wait delays (e.g. drivers/sdhci.rs's
/// back-to-back register write spacing workaround) rather than the
/// interrupt-driven virtual timer set up in drivers/generic_timer.rs.
pub fn get_cntpct_el0() -> u64 {
    let cntpct_el0: u64;
    unsafe { asm!("mrs {}, cntpct_el0", out(reg) cntpct_el0) };
    cntpct_el0
}

/// Reads the generic timer's counter frequency (Hz), as fixed by firmware.
pub fn get_cntfrq_el0() -> u64 {
    let cntfrq_el0: u64;
    unsafe { asm!("mrs {}, cntfrq_el0", out(reg) cntfrq_el0) };
    cntfrq_el0
}

pub fn get_id_aa64mmfr0_el1() -> u64 {
    let id_aa64mmfr0_el1: u64;
    unsafe { asm!("mrs {}, id_aa64mmfr0_el1", out(reg) id_aa64mmfr0_el1) };
    id_aa64mmfr0_el1
}

pub fn get_vtcr_el2() -> u64 {
    let vtcr_el2: u64;
    unsafe { asm!("mrs {}, vtcr_el2", out(reg) vtcr_el2) };
    vtcr_el2
}

pub unsafe fn set_vtcr_el2(vtcr_el2: u64) {
    unsafe { asm!("msr vtcr_el2, {}", in(reg) vtcr_el2) };
}

pub fn get_vttbr_el2() -> u64 {
    let vttbr_el2: u64;
    unsafe { asm!("mrs {}, vttbr_el2", out(reg) vttbr_el2) };
    vttbr_el2
}

pub unsafe fn set_vttbr_el2(vttbr_el2: u64) {
    unsafe { asm!("msr vttbr_el2, {}", in(reg) vttbr_el2) };
}

pub fn flush_tlb_el1() {
    unsafe {
        asm!(
            "
            dsb ishst
            tlbi alle1is
            "
        );
    }
}

pub unsafe fn set_vbar_el2(vbar_el2: u64) {
    unsafe { asm!("msr vbar_el2, {}", in(reg) vbar_el2) };
}

pub fn get_elr_el2() -> u64 {
    let elr_el2: u64;
    unsafe { asm!("mrs {}, elr_el2", out(reg) elr_el2) };
    elr_el2
}

pub unsafe fn advance_elr_el2() {
    unsafe { set_elr_el2(get_elr_el2() + 4) }
}

pub fn get_esr_el2() -> u64 {
    let esr_el2: u64;
    unsafe { asm!("mrs {}, esr_el2", out(reg) esr_el2) };
    esr_el2
}

pub fn get_far_el2() -> u64 {
    let far_el2: u64;
    unsafe { asm!("mrs {}, far_el2", out(reg) far_el2) };
    far_el2
}

pub fn get_hpfar_el2() -> u64 {
    let hpfar_el2: u64;
    unsafe { asm!("mrs {}, hpfar_el2", out(reg) hpfar_el2) };
    hpfar_el2
}

pub fn get_mpidr_el1() -> u64 {
    let mpidr_el1: u64;
    unsafe { asm!("mrs {}, mpidr_el1", out(reg) mpidr_el1) };
    mpidr_el1
}

pub const fn mpidr_to_affinity(mpidr: u64) -> u64 {
    mpidr & !((1 << 31) | (1 << 30))
}

pub unsafe fn invalidate_cache(address: usize) {
    unsafe { asm!("dc ivac, {}", in(reg) address) };
}

/// Clean the D-cache to the point of unification and invalidate the
/// I-cache for `[address, address + size)`.
///
/// This must be called after writing executable code (e.g. a guest kernel
/// image) into memory via normal cached stores and before that code is
/// executed, since a CPU with real (non-modeled) caches is not guaranteed
/// to observe the freshly written bytes through the instruction fetch path
/// otherwise. Without this, the guest may fetch stale/garbage instructions
/// on real hardware even though the same code works under QEMU (which does
/// not model cache incoherency).
pub unsafe fn clean_dcache_and_invalidate_icache(address: usize, size: usize) {
    let ctr_el0: u64;
    unsafe { asm!("mrs {}, ctr_el0", out(reg) ctr_el0) };
    let dcache_line_size: usize = 4usize << ((ctr_el0 >> 16) & 0xF);
    let icache_line_size: usize = 4usize << (ctr_el0 & 0xF);

    let end = address + size;

    /* Clean each D-cache line covering the range to the point of unification. */
    let mut addr = address & !(dcache_line_size - 1);
    while addr < end {
        unsafe { asm!("dc cvau, {}", in(reg) addr) };
        addr += dcache_line_size;
    }
    unsafe { asm!("dsb ish") };

    /* Invalidate each I-cache line covering the range to the point of unification. */
    let mut addr = address & !(icache_line_size - 1);
    while addr < end {
        unsafe { asm!("ic ivau, {}", in(reg) addr) };
        addr += icache_line_size;
    }
    unsafe { asm!("dsb ish") };
    unsafe { asm!("isb") };
}

pub fn get_midr_el1() -> u64 {
    let midr_el1: u64;
    unsafe { asm!("mrs {}, midr_el1", out(reg) midr_el1) };
    midr_el1
}

pub unsafe fn set_vmpidr_el2(vmpidr_el2: u64) {
    unsafe { asm!("msr vmpidr_el2, {}", in(reg) vmpidr_el2) };
}

pub unsafe fn set_vpidr_el2(vpidr_el2: u64) {
    unsafe { asm!("msr vpidr_el2, {}", in(reg) vpidr_el2) };
}

pub unsafe fn set_cntvoff_el2(cntvoff_el2: u64) {
    unsafe { asm!("msr cntvoff_el2, {}", in(reg) cntvoff_el2) };
}

pub unsafe fn smc(mut x0: u64, x1: u64, x2: u64, x3: u64) -> u64 {
    unsafe {
        asm!("smc 0",
        inout("x0") x0,
        in("x1") x1,
        in("x2") x2,
        in("x3") x3,
        clobber_abi("C")
        )
    };
    x0
}

#[unsafe(naked)]
pub extern "C" fn core_entry() -> ! {
    naked_asm!("
            mov sp, x0
            b   {}",
        sym crate::core_main
    )
}

/// Data Synchronization Barrier (waits for prior memory accesses to complete).
pub unsafe fn dsb_sy() {
    unsafe { asm!("dsb sy") };
}

/// Signals an event, used to wake up CPUs parked in a `wfe` spin loop
/// (e.g. the platform firmware's ARM "spin-table" boot protocol holding pen).
pub unsafe fn sev() {
    unsafe { asm!("sev") };
}

/// Entry point for CPUs woken up through the ARM "spin-table" boot protocol
/// (used, e.g., by Raspberry Pi 4's firmware instead of PSCI). Unlike
/// [`core_entry`], no register is guaranteed to hold a usable value when the
/// firmware's holding pen jumps here, so the stack pointer is instead loaded
/// from [`crate::psci::SPIN_TABLE_SP`], which must be written by the CPU
/// bringing this core up *before* arming the spin-table release address.
#[unsafe(naked)]
pub extern "C" fn spin_table_entry() -> ! {
    naked_asm!("
            adrp x0, {sp}
            add  x0, x0, :lo12:{sp}
            ldr  x0, [x0]
            mov  sp, x0
            b    {main}",
        sp = sym crate::psci::SPIN_TABLE_SP,
        main = sym crate::core_main,
    )
}

pub unsafe fn get_daif_and_disable_irq_fiq() -> u64 {
    let daif: u64;
    unsafe {
        asm!("
            mrs {t},    daif
            mov {r},    {t}
            orr {t},    {t}, ( 1 << 7 /* IRQ */ ) | ( 1 << 6 /* FIQ */ )
            msr daif,   {t}
            isb",
        t = out(reg) _ ,
        r = out(reg) daif
        )
    };
    daif
}

pub unsafe fn set_daif(daif: u64) {
    unsafe {
        asm!("
            isb
            msr daif, {}",
        in(reg) daif
        )
    };
}

pub fn get_tpidr_el2() -> u64 {
    let tpidr_el2: u64;
    unsafe { asm!("mrs {}, tpidr_el2", out(reg) tpidr_el2) };
    tpidr_el2
}

pub unsafe fn set_tpidr_el2(tpidr_el2: u64) {
    unsafe { asm!("msr tpidr_el2, {}", in(reg) tpidr_el2) };
}
