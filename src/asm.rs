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

pub fn get_spsr_el2() -> u64 {
    let spsr_el2: u64;
    unsafe { asm!("mrs {}, spsr_el2", out(reg) spsr_el2) };
    spsr_el2
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

pub fn get_dcache_line_size() -> usize {
    let ctr_el0: u64;
    unsafe { asm!("mrs {}, ctr_el0", out(reg) ctr_el0) };
    4usize << ((ctr_el0 >> 16) & 0xF)
}

/// Cleans the D-cache for `[address, address + size)` to the Point of
/// Coherency.
///
/// The trailing barrier is `dsb sy` (full system), NOT `dsb ish` (inner
/// shareable), because this routine is used to make CPU writes visible to an
/// external, non-I/O-coherent DMA master -- the Raspberry Pi 4's VL805 xHCI
/// controller reached over PCIe. That master lives outside the CPU's inner
/// shareable domain, so an `dsb ish` may return before the cleaned lines have
/// actually drained through the interconnect to the Point of Coherency that
/// the PCIe master observes. Using `dsb ish` here caused the controller to
/// DMA-read stale/garbage command-ring, DCBAA and ERST contents, raising a
/// Host System Error (USBSTS.HSE) as soon as the first doorbell was rung.
/// Both U-Boot (`__asm_flush_dcache_range`) and Linux (`dcache_clean_poc`)
/// use `dsb sy` for exactly this reason.
pub unsafe fn clean_dcache_range(address: usize, size: usize) {
    let line_size = get_dcache_line_size();
    let mut addr = address & !(line_size - 1);
    let end = address.saturating_add(size);
    while addr < end {
        unsafe { asm!("dc cvac, {}", in(reg) addr) };
        addr += line_size;
    }
    unsafe { asm!("dsb sy") };
}

/// Invalidates the D-cache for `[address, address + size)` to the Point of
/// Coherency.
///
/// Uses `dsb sy` (full system) rather than `dsb ish` for the same reason as
/// [`clean_dcache_range`]: this is used to discard stale CPU cache lines
/// before reading data produced by the external PCIe DMA master, which is not
/// part of the inner shareable domain.
pub unsafe fn invalidate_dcache_range(address: usize, size: usize) {
    let line_size = get_dcache_line_size();
    let mut addr = address & !(line_size - 1);
    let end = address.saturating_add(size);
    while addr < end {
        unsafe { asm!("dc ivac, {}", in(reg) addr) };
        addr += line_size;
    }
    unsafe { asm!("dsb sy") };
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

/// Entry point for a physical CPU brought up (via PSCI CPU_ON) by
/// [`main::park_secondary_cpus_for_smp`], to be parked at EL2 as an
/// additional vCPU available to the *same* guest VM, rather than
/// [`core_entry`]'s "start a brand-new, independent VM" behavior.
#[unsafe(naked)]
pub extern "C" fn smp_park_entry() -> ! {
    naked_asm!("
            mov sp, x0
            b   {}",
        sym crate::smp_park_main
    )
}

/// Re-parks the current physical CPU (see `main::smp_park_main`) after a
/// guest-issued PSCI CPU_OFF has retired the VCPU it was running (see
/// `main::handle_guest_cpu_off`), when no other queued VCPU on this same
/// pCPU is available to switch to immediately.
///
/// Resets `sp` to `stack_top` (the same per-pCPU stack top address this
/// physical CPU was originally started on, e.g. by
/// `main::park_secondary_cpus_for_smp`) before re-entering
/// [`crate::smp_park_main`], rather than simply calling it as an ordinary
/// Rust function from deep inside the CPU_OFF trap's call stack. Re-running
/// `smp_park_main`'s (idempotent) setup is harmless, but never popping the
/// exception frame and call stack accumulated up to that point would leak
/// a little of this pCPU's stack on every such CPU_OFF/CPU_ON cycle,
/// eventually overflowing it (e.g. under a guest that repeatedly
/// hotplugs/hotunplugs the same vCPU) -- resetting `sp` here avoids that
/// entirely, exactly like this physical CPU's original park entry
/// ([`smp_park_entry`]) does.
#[unsafe(naked)]
pub extern "C" fn reset_stack_and_park(_stack_top: u64) -> ! {
    naked_asm!("
            mov sp, x0
            b   {}",
        sym crate::smp_park_main
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

/// Waits for an event (typically a subsequent [`sev`] from another core),
/// or returns immediately if one is already pending. Used to park a
/// physical CPU cheaply (instead of a hot `spin_loop`) while it waits to be
/// handed a guest vCPU to run (see `main::smp_park_main`).
pub fn wfe() {
    unsafe { asm!("wfe") };
}

/// Entry point for CPUs woken up through the ARM "spin-table" boot protocol
/// (used, e.g., by Raspberry Pi 4's firmware instead of PSCI). Unlike
/// [`core_entry`], no register is guaranteed to hold a usable value when the
/// firmware's holding pen jumps here, so the stack pointer is instead loaded
/// from [`crate::psci::SPIN_TABLE_SP`], which must be written by the CPU
/// bringing this core up *before* arming the spin-table release address.
/// Which Rust function it ultimately jumps to (e.g. [`crate::core_main`]
/// or [`crate::smp_park_main`]) is likewise not known statically -- the
/// spin-table release address can only ever hold this one fixed trampoline
/// -- so it is loaded from [`crate::psci::SPIN_TABLE_TARGET`] and reached
/// via an indirect branch instead.
#[unsafe(naked)]
pub extern "C" fn spin_table_entry() -> ! {
    naked_asm!("
            adrp x0, {sp}
            add  x0, x0, :lo12:{sp}
            ldr  x0, [x0]
            mov  sp, x0
            adrp x1, {target}
            add  x1, x1, :lo12:{target}
            ldr  x1, [x1]
            br   x1",
        sp = sym crate::psci::SPIN_TABLE_SP,
        target = sym crate::psci::SPIN_TABLE_TARGET,
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

/// Generates a `get_<name>`/`set_<name>` pair for a guest EL0/EL1 system
/// register. Unlike `HCR_EL2`/`VTCR_EL2`/etc. above, these registers are
/// banked *per Exception Level* by the hardware, not per-VM: only one
/// "copy" of each physically exists, shared by every VCPU that ever runs
/// at EL1 on this pCPU. Cooperatively switching between multiple VCPUs
/// scheduled on the same pCPU (see `vm::try_yield_to_next_vcpu`) therefore
/// requires explicitly saving and restoring every one of them (MMU/cache
/// configuration, exception vectors, banked stack pointers, the virtual
/// timer comparator, etc.) as part of that VCPU's off-CPU context (see
/// `vm::VcpuContext`) -- otherwise an incoming VCPU would keep running
/// with whatever the outgoing VCPU had last programmed into these
/// registers (e.g. its MMU translation tables), rather than its own.
macro_rules! el1_register_accessors {
    ($reg:ident, $get:ident, $set:ident) => {
        pub fn $get() -> u64 {
            let value: u64;
            unsafe { asm!(concat!("mrs {}, ", stringify!($reg)), out(reg) value) };
            value
        }

        pub unsafe fn $set(value: u64) {
            unsafe { asm!(concat!("msr ", stringify!($reg), ", {}"), in(reg) value) };
        }
    };
}

el1_register_accessors!(sctlr_el1, get_sctlr_el1, set_sctlr_el1);
el1_register_accessors!(ttbr0_el1, get_ttbr0_el1, set_ttbr0_el1);
el1_register_accessors!(ttbr1_el1, get_ttbr1_el1, set_ttbr1_el1);
el1_register_accessors!(tcr_el1, get_tcr_el1, set_tcr_el1);
el1_register_accessors!(mair_el1, get_mair_el1, set_mair_el1);
el1_register_accessors!(amair_el1, get_amair_el1, set_amair_el1);
el1_register_accessors!(vbar_el1, get_vbar_el1, set_vbar_el1);
el1_register_accessors!(cpacr_el1, get_cpacr_el1, set_cpacr_el1);
el1_register_accessors!(contextidr_el1, get_contextidr_el1, set_contextidr_el1);
el1_register_accessors!(esr_el1, get_esr_el1, set_esr_el1);
el1_register_accessors!(far_el1, get_far_el1, set_far_el1);
el1_register_accessors!(par_el1, get_par_el1, set_par_el1);
el1_register_accessors!(afsr0_el1, get_afsr0_el1, set_afsr0_el1);
el1_register_accessors!(afsr1_el1, get_afsr1_el1, set_afsr1_el1);
el1_register_accessors!(tpidr_el0, get_tpidr_el0, set_tpidr_el0);
el1_register_accessors!(tpidr_el1, get_tpidr_el1, set_tpidr_el1);
el1_register_accessors!(sp_el0, get_sp_el0, set_sp_el0);
el1_register_accessors!(sp_el1, get_sp_el1, set_sp_el1);
el1_register_accessors!(elr_el1, get_elr_el1, set_elr_el1);
el1_register_accessors!(spsr_el1, get_spsr_el1, set_spsr_el1);
el1_register_accessors!(cntkctl_el1, get_cntkctl_el1, set_cntkctl_el1);
el1_register_accessors!(cntv_ctl_el0, get_cntv_ctl_el0, set_cntv_ctl_el0);
el1_register_accessors!(cntv_cval_el0, get_cntv_cval_el0, set_cntv_cval_el0);
