//!
//! Module for enumerating register constant values
//!

/* HCR_EL2 */
pub const HCR_EL2_API: u64 = 1 << 41;
pub const HCR_EL2_RW: u64 = 1 << 31;
/// Trap guest (EL1/EL0) WFI to EL2. On real Raspberry Pi 4 hardware the physical
/// PL011 RX interrupt (SPI 153) stays in GIC Group 0 and is never delivered to the
/// Non-secure EL2 hypervisor (confirmed: neither fiq_handler nor irq_handler ever
/// runs for it), so interrupt-driven console input is impossible. Trapping WFI gives
/// a reliable hook to poll the physical UART: while the guest idles at a prompt it
/// executes WFI, which traps here, letting us drain the physical RX FIFO and inject
/// any keystrokes into the guest's virtual PL011 before resuming.
pub const HCR_EL2_TWI: u64 = 1 << 13;
pub const HCR_EL2_AMO: u64 = 1 << 5;
pub const HCR_EL2_IMO: u64 = 1 << 4;
pub const HCR_EL2_FMO: u64 = 1 << 3;
pub const HCR_EL2_VM: u64 = 1 << 0;
/// Trap guest (EL1/EL0) SMC instructions to EL2 instead of EL3. This DTB's
/// `psci` node advertises `method = "smc"`, so the guest kernel invokes
/// PSCI (including its own CPU_ON calls to bring up additional vCPUs) via
/// SMC; trapping it here lets the hypervisor virtualize CPU_ON for true
/// guest SMP (see `main::handle_guest_smc`) while transparently forwarding
/// every other PSCI call to the real firmware.
pub const HCR_EL2_TSC: u64 = 1 << 19;

/* SPSR_EL2 */
pub const SPSR_EL2_M_EL1H: u64 = 0b0101;

/* VTTBR_EL2 */
pub const VTTBR_BADDR: u64 = ((1 << 47) - 1) & !1;
/// 8-bit VMID field (valid when VTCR_EL2.VS == 0, i.e. an 8-bit VMID, which
/// is what `paging::create_stage2_translation_table` configures). Tags
/// Stage 2 (and combined Stage 1+2) TLB entries so that multiple VCPUs
/// scheduled on the same pCPU, each with their own Stage 2 translation
/// table, can share the physical TLB without their entries colliding or
/// needing an explicit flush on every VCPU switch.
pub const VTTBR_VMID_BITS_OFFSET: u64 = 48;
pub const VTTBR_VMID: u64 = 0xFF << VTTBR_VMID_BITS_OFFSET;

/* VTCR_EL2 */
pub const VTCR_EL2_RES1: u64 = 1 << 31;
pub const VTCR_EL2_PS_BITS_OFFSET: u64 = 16;
pub const VTCR_EL2_TG0_BITS_OFFSET: u64 = 14;
pub const VTCR_EL2_SH0_BITS_OFFSET: u64 = 12;
pub const VTCR_EL2_ORGN0_BITS_OFFSET: u64 = 10;
pub const VTCR_EL2_IRGN0_BITS_OFFSET: u64 = 8;
pub const VTCR_EL2_SL0_BITS_OFFSET: u64 = 6;
pub const VTCR_EL2_SL0: u64 = 0b11 << VTCR_EL2_SL0_BITS_OFFSET;
pub const VTCR_EL2_T0SZ_BITS_OFFSET: u64 = 0;
pub const VTCR_EL2_T0SZ: u64 = 0b111111 << VTCR_EL2_T0SZ_BITS_OFFSET;

/* ID_AA64MMFR0_EL1 */
pub const ID_AA64MMFR0_EL1_PARANGE: u64 = 0b1111;

/* ESR_EL2 */
pub const ESR_EL2_EC_BITS_OFFSET: u64 = 26;
pub const ESR_EL2_EC: u64 = 0b111111 << ESR_EL2_EC_BITS_OFFSET;
pub const ESR_EL2_EC_DATA_ABORT: u64 = 0b100100 << 26;
/// EC value for a trapped WFI/WFE instruction (Arm ARM: "Trapped WFI or WFE"). Used
/// with HCR_EL2.TWI so guest WFI becomes a UART-polling hook (see HCR_EL2_TWI).
pub const ESR_EL2_EC_WFX: u64 = 0b000001 << 26;
/// EC value for a trapped SMC instruction executed in AArch64 state (Arm
/// ARM: "SMC instruction execution in AArch64 state"). Used with
/// HCR_EL2.TSC to virtualize the guest's own PSCI calls (see
/// `main::handle_guest_smc`), most importantly CPU_ON for guest SMP.
pub const ESR_EL2_EC_SMC64: u64 = 0b010111 << 26;
pub const ESR_EL2_ISS_ISV: u64 = 1 << 24;
pub const ESR_EL2_ISS_SAS_BITS_OFFSET: u64 = 22;
pub const ESR_EL2_ISS_SAS: u64 = 0b11 << ESR_EL2_ISS_SAS_BITS_OFFSET;
pub const ESR_EL2_ISS_SRT_BITS_OFFSET: u64 = 16;
pub const ESR_EL2_ISS_SRT: u64 = 0b11111 << ESR_EL2_ISS_SRT_BITS_OFFSET;
pub const ESR_EL2_ISS_SF: u64 = 1 << 15;
pub const ESR_EL2_ISS_WNR: u64 = 1 << 6;

/* HPFAR_EL2 */
pub const HPFAR_EL2_FIPA_BITS_OFFSET: u64 = 4;
pub const HPFAR_EL2_FIPA: u64 = ((1 << 44) - 1) & !((1 << 4) - 1);
