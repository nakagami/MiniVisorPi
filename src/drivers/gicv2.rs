//!
//! GICv2 interrupt management module
//!

pub const DTB_GIC_LEVEL: u32 = 4;
pub const DTB_GIC_SPI: u32 = 0;
pub const DTB_GIC_PPI: u32 = 1;
pub const GIC_PPI_BASE: u32 = 16;
pub const GIC_SPI_BASE: u32 = 32;

#[derive(Copy, Clone, Eq, PartialEq)]
pub enum GicGroup {
    NonSecureGroup1,
}

//
// GICv2 consists only of memory-mapped registers and has no system registers
// (ICC_*/ICH_*). Keep the base addresses of the physical Distributor / CPU Interface /
// Hypervisor Interface so that each core's interrupt handler and the vGIC can reference them.
// (Every core always accesses the same physical address, but the hardware treats
//  them as banked registers per accessing core.)
//
static mut GICD_BASE_ADDRESS: usize = 0;
static mut GICC_BASE_ADDRESS: usize = 0;
static mut GICH_BASE_ADDRESS: usize = 0;

pub struct GicDistributor {
    base_address: usize,
}

pub struct GicCpuInterface {
    base_address: usize,
}

pub struct GicHypervisorInterface {
    base_address: usize,
}

impl GicDistributor {
    pub const GICD_MMIO_SIZE: usize = 0x1000;

    const GICD_CTLR: usize = 0x000;
    const GICD_CTLR_ENABLE_GRP0: u32 = 1 << 0;
    const GICD_CTLR_ENABLE_GRP1: u32 = 1 << 1;
    const GICD_IGROUPR: usize = 0x080;
    const GICD_ISENABLER: usize = 0x100;
    const GICD_ICENABLER: usize = 0x180;
    const GICD_ISPENDR: usize = 0x200;
    const GICD_ICPENDR: usize = 0x280;
    const GICD_IPRIORITYR: usize = 0x400;
    const GICD_ITARGETSR: usize = 0x800;
    const GICD_ICFGR: usize = 0xC00;
    const GICD_SGIR: usize = 0xF00;

    pub fn new(base_address: usize, size: usize) -> Result<Self, ()> {
        if size < Self::GICD_MMIO_SIZE {
            println!("Invalid GICD Size: {:#X}", size);
            return Err(());
        }
        unsafe { GICD_BASE_ADDRESS = base_address };
        Ok(Self { base_address })
    }

    pub fn init(&self) {
        self.write_register(
            Self::GICD_CTLR,
            Self::GICD_CTLR_ENABLE_GRP0 | Self::GICD_CTLR_ENABLE_GRP1,
        );
    }

    pub fn set_priority(&self, int_id: u32, priority: u8) {
        let register_index = ((int_id >> 2) as usize) * size_of::<u32>();
        let register_offset = (int_id & 0b11) << 3;
        self.write_register(
            Self::GICD_IPRIORITYR + register_index,
            (self.read_register(Self::GICD_IPRIORITYR + register_index)
                & !(0xFF << register_offset))
                | ((priority as u32) << register_offset),
        );
    }

    pub fn set_group(&self, int_id: u32, group: GicGroup) {
        let register_index = ((int_id / u32::BITS) as usize) * size_of::<u32>();
        let register_offset = int_id & (u32::BITS - 1);
        let data = match group {
            GicGroup::NonSecureGroup1 => 1,
        };
        self.write_register(
            Self::GICD_IGROUPR + register_index,
            (self.read_register(Self::GICD_IGROUPR + register_index) & !(1 << register_offset))
                | (data << register_offset),
        );
    }

    pub fn set_enable(&self, int_id: u32, enable: bool) {
        let register_index = ((int_id / u32::BITS) as usize) * size_of::<u32>();
        let register_offset = int_id & (u32::BITS - 1);
        let register = if enable {
            Self::GICD_ISENABLER
        } else {
            Self::GICD_ICENABLER
        };

        self.write_register(register + register_index, 1 << register_offset);
    }

    pub fn set_pending(&self, int_id: u32, pending: bool) {
        let register_index = ((int_id / u32::BITS) as usize) * size_of::<u32>();
        let register_offset = int_id & (u32::BITS - 1);
        let register = if pending {
            Self::GICD_ISPENDR
        } else {
            Self::GICD_ICPENDR
        };
        self.write_register(
            register + register_index,
            self.read_register(register + register_index) | (1 << register_offset),
        );
    }

    pub fn set_trigger_mode(&self, int_id: u32, is_level_trigger: bool) {
        let register_index = ((int_id / (u32::BITS / 2)) as usize) * size_of::<u32>();
        let register_offset = (int_id & (u32::BITS / 2 - 1)) * 2;

        self.write_register(
            Self::GICD_ICFGR + register_index,
            (self.read_register(Self::GICD_ICFGR + register_index) & !(0x03 << register_offset))
                | ((((!is_level_trigger) as u32) << 1) << register_offset),
        );
    }

    /// Sets the interrupt's destination (CPU Targets List).
    /// GICv2 specifies the destination with an 8-bit CPU ID bitmask instead of Affinity.
    pub fn set_target(&self, int_id: u32, target: u8) {
        let register_index = ((int_id >> 2) as usize) * size_of::<u32>();
        let register_offset = (int_id & 0b11) << 3;
        self.write_register(
            Self::GICD_ITARGETSR + register_index,
            (self.read_register(Self::GICD_ITARGETSR + register_index)
                & !(0xFF << register_offset))
                | ((target as u32) << register_offset),
        );
    }

    /// Gets the target bit of the currently running physical CPU itself.
    /// (ITARGETSR for interrupts 0-31 is banked, so the accessing CPU reads its own bit)
    pub fn get_own_target(&self) -> u8 {
        (self.read_register(Self::GICD_ITARGETSR) & 0xFF) as u8
    }

    /// Reads back an SPI's group/enable/target/priority/config bits and prints them.
    /// Used to confirm on real hardware whether writes issued by `set_group`/`set_enable`/
    /// `set_target`/`set_priority`/`set_trigger_mode` actually stuck (e.g. some GIC
    /// implementations RAZ/WI GICD_IGROUPR writes from Non-secure state when the interrupt
    /// was not already assigned to Non-secure Group 1 by secure-world firmware).
    pub fn dump_spi_config(&self, int_id: u32) {
        let group_reg = self.read_register(
            Self::GICD_IGROUPR + ((int_id / u32::BITS) as usize) * size_of::<u32>(),
        );
        let group_bit = (group_reg >> (int_id & (u32::BITS - 1))) & 1;
        let enable_reg = self.read_register(
            Self::GICD_ISENABLER + ((int_id / u32::BITS) as usize) * size_of::<u32>(),
        );
        let enable_bit = (enable_reg >> (int_id & (u32::BITS - 1))) & 1;
        let target = (self.read_register(
            Self::GICD_ITARGETSR + ((int_id >> 2) as usize) * size_of::<u32>(),
        ) >> ((int_id & 0b11) << 3))
            & 0xFF;
        let priority = (self.read_register(
            Self::GICD_IPRIORITYR + ((int_id >> 2) as usize) * size_of::<u32>(),
        ) >> ((int_id & 0b11) << 3))
            & 0xFF;
        let cfg = (self.read_register(
            Self::GICD_ICFGR + ((int_id / (u32::BITS / 2)) as usize) * size_of::<u32>(),
        ) >> ((int_id & (u32::BITS / 2 - 1)) * 2))
            & 0b11;
        println!(
            "GICD SPI {int_id}: group={group_bit} enable={enable_bit} target={target:#04X} \
             priority={priority:#04X} cfg={cfg:#04b}"
        );
    }

    fn read_register(&self, register: usize) -> u32 {
        unsafe { core::ptr::read_volatile((self.base_address + register) as *const u32) }
    }

    fn write_register(&self, register: usize, data: u32) {
        unsafe { core::ptr::write_volatile((self.base_address + register) as *mut u32, data) }
    }
}

/// Gets the target bit of the currently running CPU itself (for use from places, such as
/// the vGIC, that do not hold a distributor instance).
pub fn get_current_cpu_target() -> u8 {
    let base_address = unsafe { GICD_BASE_ADDRESS };
    (unsafe { core::ptr::read_volatile((base_address + GicDistributor::GICD_ITARGETSR) as *const u32) }
        & 0xFF) as u8
}

/// Sends an SGI to the specified target CPU.
pub fn send_sgi(target: u8, sgi_id: u32) {
    let base_address = unsafe { GICD_BASE_ADDRESS };
    let value = (sgi_id & 0xF) | ((target as u32) << 16);
    unsafe {
        core::ptr::write_volatile(
            (base_address + GicDistributor::GICD_SGIR) as *mut u32,
            value,
        )
    };
}

impl GicCpuInterface {
    pub const GICC_MMIO_SIZE: usize = 0x1000;
    pub const DEFAULT_PRIORITY_MASK: u8 = 0xff;
    pub const DEFAULT_BINARY_POINT: u8 = 0x03;

    const GICC_CTLR: usize = 0x000;
    const GICC_CTLR_ENABLE_GRP0: u32 = 1 << 0;
    const GICC_CTLR_ENABLE_GRP1: u32 = 1 << 1;
    /// Bit that allows acknowledging Group 1 interrupts from a secure access.
    /// (In GICv2 with QEMU's `security_extn=false` (secure=off), the only CPU interface
    ///  access, which is treated as a secure access, requires this to be able to ack Group 1 interrupts)
    const GICC_CTLR_ACK_CTL: u32 = 1 << 2;
    const GICC_CTLR_EOI_MODE_NS: u32 = 1 << 9;
    const GICC_PMR: usize = 0x004;
    const GICC_BPR: usize = 0x008;
    const GICC_IAR: usize = 0x00C;
    const GICC_EOIR: usize = 0x010;
    const GICC_IAR_INT_ID: u32 = (1 << 10) - 1;
    const GICC_DIR: usize = 0x1000;
    /// Value read back from GICC_IAR's INT_ID field when no interrupt is
    /// pending for this CPU interface (GICv2 spec, ID 1023).
    pub const SPURIOUS_INT_ID: u32 = 1023;

    pub fn new(base_address: usize) -> Self {
        unsafe { GICC_BASE_ADDRESS = base_address };
        Self { base_address }
    }

    pub fn init(&self) {
        self.write_register(Self::GICC_PMR, Self::DEFAULT_PRIORITY_MASK as u32);
        self.write_register(Self::GICC_BPR, Self::DEFAULT_BINARY_POINT as u32);
        self.write_register(
            Self::GICC_CTLR,
            Self::GICC_CTLR_ENABLE_GRP0
                | Self::GICC_CTLR_ENABLE_GRP1
                | Self::GICC_CTLR_ACK_CTL
                | Self::GICC_CTLR_EOI_MODE_NS,
        );
    }

    pub fn get_acknowledge() -> (u32, GicGroup) {
        let base_address = unsafe { GICC_BASE_ADDRESS };
        let iar =
            unsafe { core::ptr::read_volatile((base_address + Self::GICC_IAR) as *const u32) };
        (iar & Self::GICC_IAR_INT_ID, GicGroup::NonSecureGroup1)
    }

    pub fn drop_priority(int_id: u32, _group: GicGroup) {
        let base_address = unsafe { GICC_BASE_ADDRESS };
        unsafe {
            core::ptr::write_volatile((base_address + Self::GICC_EOIR) as *mut u32, int_id)
        };
    }

    pub fn deactivate(int_id: u32) {
        let base_address = unsafe { GICC_BASE_ADDRESS };
        unsafe { core::ptr::write_volatile((base_address + Self::GICC_DIR) as *mut u32, int_id) };
    }

    fn write_register(&self, register: usize, data: u32) {
        unsafe { core::ptr::write_volatile((self.base_address + register) as *mut u32, data) }
    }
}

impl GicHypervisorInterface {
    pub const GICH_MMIO_SIZE: usize = 0x1000;

    const GICH_HCR: usize = 0x000;
    const GICH_HCR_EN: u32 = 1 << 0;
    const GICH_VTR: usize = 0x004;
    const GICH_VTR_LIST_REGS: u32 = 0b11_1111;
    const GICH_EISR0: usize = 0x020;
    const GICH_LR0: usize = 0x100;

    pub fn new(base_address: usize) -> Self {
        unsafe { GICH_BASE_ADDRESS = base_address };
        Self { base_address }
    }

    pub fn init(&self) {
        self.write_register(Self::GICH_HCR, Self::GICH_HCR_EN);
    }

    fn write_register(&self, register: usize, data: u32) {
        unsafe { core::ptr::write_volatile((self.base_address + register) as *mut u32, data) }
    }
}

/* Functions for accessing GICH (Hypervisor Control) from vgic.rs */

pub fn get_gich_vtr_list_regs() -> usize {
    let base_address = unsafe { GICH_BASE_ADDRESS };
    ((unsafe { core::ptr::read_volatile((base_address + GicHypervisorInterface::GICH_VTR) as *const u32) }
        & GicHypervisorInterface::GICH_VTR_LIST_REGS) as usize)
        + 1
}

pub fn get_gich_eisr() -> u32 {
    let base_address = unsafe { GICH_BASE_ADDRESS };
    unsafe {
        core::ptr::read_volatile((base_address + GicHypervisorInterface::GICH_EISR0) as *const u32)
    }
}

pub fn get_gich_lr(index: usize) -> u32 {
    let base_address = unsafe { GICH_BASE_ADDRESS };
    unsafe {
        core::ptr::read_volatile(
            (base_address + GicHypervisorInterface::GICH_LR0 + index * size_of::<u32>())
                as *const u32,
        )
    }
}

pub fn set_gich_lr(index: usize, value: u32) {
    let base_address = unsafe { GICH_BASE_ADDRESS };
    unsafe {
        core::ptr::write_volatile(
            (base_address + GicHypervisorInterface::GICH_LR0 + index * size_of::<u32>())
                as *mut u32,
            value,
        )
    };
}
