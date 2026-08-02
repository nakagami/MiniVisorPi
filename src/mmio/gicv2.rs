//!
//!  Generic Interrupt Controller version 2 MMIO Driver
//!

use crate::drivers::gicv2;
use crate::vgic;
use crate::vm;
use crate::vm::MmioHandler;

use alloc::collections::linked_list::LinkedList;

/* GIC Distributor (Virtual) */
/*
 * GICv2 has no Redistributor, and SGI/PPI (ID 0-31) configuration is also done via
 * registers in the same MMIO region as the Distributor (banked per accessing CPU),
 * so SPI and SGI/PPI are handled together in a single struct.
 * (This hypervisor uses a 1 VM = 1 vCPU = 1 pCPU configuration, so banking need not be considered)
 */
pub struct GicDistributorMmio {
    ctlr: u32,
    group: [u32; 32],
    enable: [u32; 32],
    pending: [u32; 32],
    active: [u32; 32],
    priority: [u32; 255],
    configuration: [u32; 64],
    target: [u32; 255],
    /// GICv2 target bit of the physical CPU running this VM
    own_target: u8,
    to_inject_interrupt: LinkedList<(u8, u32)>,
}

const GIC_REVISION: u64 = 2;

/* Registers */
const GICD_CTLR: usize = 0x0000;
const GICD_TYPER: usize = 0x0004;
const GICD_IGROUPR0: usize = 0x0080;
const GICD_IGROUPR31: usize = 0x00FC;
const GICD_ISENABLER0: usize = 0x0100;
const GICD_ISENABLER31: usize = 0x017C;
const GICD_ICENABLER0: usize = 0x0180;
const GICD_ICENABLER31: usize = 0x01FC;
const GICD_ISPENDR0: usize = 0x0200;
const GICD_ISPENDR31: usize = 0x027C;
const GICD_ICPENDR0: usize = 0x0280;
const GICD_ICPENDR31: usize = 0x02FC;
const GICD_ISACTIVER0: usize = 0x0300;
const GICD_ISACTIVER31: usize = 0x037C;
const GICD_ICACTIVER0: usize = 0x0380;
const GICD_ICACTIVER31: usize = 0x03FC;
const GICD_IPRIORITYR0: usize = 0x0400;
const GICD_IPRIORITYR254: usize = 0x07F8;
const GICD_ITARGETSR0: usize = 0x0800;
const GICD_ITARGETSR254: usize = 0x0BF8;
const GICD_ICFGR0: usize = 0x0C00;
const GICD_ICFGR63: usize = 0x0CFC;
const GICD_SGIR: usize = 0x0F00;
const GICD_PIDR2: usize = 0xFFE8;

const GICD_CTLR_ENABLE_GRP0: u32 = 1 << 0;
const GICD_CTLR_ENABLE_GRP1: u32 = 1 << 1;

const GICD_TYPER_VALUE: u32 = 31; /* ITLinesNumber: Max SPI ID (1023) */

pub const INJECT_INTERRUPT_INT_ID: u32 = 11;

impl GicDistributorMmio {
    pub const MMIO_SIZE: usize = 0x10000;

    pub fn new(own_target: u8) -> Self {
        Self {
            ctlr: 0,
            group: [0; 32],
            enable: [0; 32],
            pending: [0; 32],
            active: [0; 32],
            priority: [0; 255],
            configuration: [0; 64],
            target: [0; 255],
            own_target,
            to_inject_interrupt: LinkedList::new(),
        }
    }

    fn get_group(&self, int_id: u32) -> u32 {
        let int_id = int_id as usize;
        const DATA_PER_REG: usize = u32::BITS as usize;
        let register = int_id / DATA_PER_REG;
        let offset = int_id % DATA_PER_REG;
        (self.group[register] >> offset) & 0b1
    }

    fn get_enable(&self, int_id: u32) -> bool {
        let int_id = int_id as usize;
        const DATA_PER_REG: usize = u32::BITS as usize;
        let register = int_id / DATA_PER_REG;
        let offset = int_id % DATA_PER_REG;
        ((self.enable[register] >> offset) & 1) != 0
    }

    fn get_pending(&self, int_id: u32) -> bool {
        let int_id = int_id as usize;
        const DATA_PER_REG: usize = u32::BITS as usize;
        let register = int_id / DATA_PER_REG;
        let offset = int_id % DATA_PER_REG;
        ((self.pending[register] >> offset) & 1) != 0
    }

    pub fn change_pending_status(&mut self, int_id: u32, is_pending: bool) {
        let int_id = int_id as usize;
        const DATA_PER_REG: usize = u32::BITS as usize;
        let register = int_id / DATA_PER_REG;
        let offset = int_id % DATA_PER_REG;
        if is_pending {
            self.pending[register] |= 1 << offset;
        } else {
            self.pending[register] &= !(1 << offset);
        }
    }

    pub fn change_active_status(&mut self, int_id: u32, is_active: bool) {
        let int_id = int_id as usize;
        const DATA_PER_REG: usize = u32::BITS as usize;
        let register = int_id / DATA_PER_REG;
        let offset = int_id % DATA_PER_REG;
        if is_active {
            self.active[register] |= 1 << offset;
        } else {
            self.active[register] &= !(1 << offset);
        }
    }

    fn get_priority(&self, int_id: u32) -> u32 {
        let int_id = int_id as usize;
        const DATA_PER_REG: usize = u32::BITS as usize / 8;
        let register = int_id / DATA_PER_REG;
        let offset = (int_id % DATA_PER_REG) * 8;
        (self.priority[register] >> offset) & 0xFF
    }

    pub fn trigger_interrupt(&mut self, int_id: u32, physical_int_id: Option<u32>) {
        self.trigger_interrupt_to(int_id, physical_int_id, self.own_target);
    }

    /// Same as [`trigger_interrupt`](Self::trigger_interrupt), but delivers to an explicit
    /// set of target pCPUs (a GICv2 8-bit CPU target bitmask) instead of always `own_target`.
    /// Used for SGIs (`GICD_SGIR`), whose destination is chosen per-write by the sending
    /// vCPU rather than being a fixed, interrupt-ID-owned target -- essential once a single
    /// VM can have multiple concurrently-running vCPUs (true guest SMP), since an SGI/IPI
    /// sent by one vCPU must reach whichever *other* vCPU(s) it targets, not just whichever
    /// pCPU originally created the VM.
    pub fn trigger_interrupt_to(&mut self, int_id: u32, physical_int_id: Option<u32>, targets: u8) {
        self.change_pending_status(int_id, true);
        /* In the single security state with security_extn=false (secure=off), only bit0
         * (Enable) of GICD_CTLR is meaningful, and Linux's GICv2 driver also only writes
         * bit0. Since bit1 (Grp1 Enable) is unused, treat it as enabled if either bit is set. */
        if (self.ctlr & (GICD_CTLR_ENABLE_GRP0 | GICD_CTLR_ENABLE_GRP1)) == 0 {
            return;
        }
        if !self.get_enable(int_id) {
            return;
        }
        let group = self.get_group(int_id);
        let priority = self.get_priority(int_id);

        let list_entry = vgic::create_list_register_entry(int_id, group, priority, physical_int_id);

        let current_target = gicv2::get_current_cpu_target();
        for bit in 0..u8::BITS {
            let target = 1u8 << bit;
            if (targets & target) == 0 {
                continue;
            }
            if target == current_target {
                /* Same pCPU: inject directly into this pCPU's own List Registers. */
                vgic::add_virtual_interrupt(list_entry);
            } else {
                /* A different pCPU (either a different VM, sharing this pCPU with the
                 * cooperative scheduler, or a different vCPU of *this* same VM, running
                 * concurrently on another physical core): queue the entry tagged with
                 * its intended destination and prod that pCPU with a physical SGI so it
                 * services just its own share of the queue. */
                self.to_inject_interrupt.push_back((target, list_entry));
                gicv2::send_sgi(target, INJECT_INTERRUPT_INT_ID);
            }
        }
    }
}

pub fn inject_interrupt_handler() {
    let vm = vm::get_current_vm();
    let mut distributor = vm.get_gic_distributor_mmio().lock();
    let current_target = gicv2::get_current_cpu_target();
    /* The queue is shared by every pCPU running a vCPU of this VM (whether that's several
     * independent VMs cooperatively sharing one pCPU, or several vCPUs of *this* VM running
     * concurrently on separate pCPUs for true SMP), so only take out and inject the entries
     * actually destined for this pCPU, leaving any others for their own target pCPU to pick
     * up when its own physical INJECT_INTERRUPT_INT_ID SGI arrives. */
    let mut remaining = LinkedList::new();
    while let Some((target, entry)) = distributor.to_inject_interrupt.pop_front() {
        if target == current_target {
            vgic::add_virtual_interrupt(entry);
        } else {
            remaining.push_back((target, entry));
        }
    }
    distributor.to_inject_interrupt = remaining;
}

impl MmioHandler for GicDistributorMmio {
    fn read(&mut self, offset: usize, access_width: u64) -> Result<u64, ()> {
        let mut result = 0u64;
        if offset == GICD_CTLR && access_width == 32 {
            result = self.ctlr as u64;
        } else if offset == GICD_TYPER && access_width == 32 {
            result = GICD_TYPER_VALUE as u64;
        } else if (GICD_IGROUPR0..=GICD_IGROUPR31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_IGROUPR0) / size_of::<u32>();
            result = self.group[register_offset] as u64;
        } else if (GICD_ISENABLER0..=GICD_ISENABLER31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ISENABLER0) / size_of::<u32>();
            result = self.enable[register_offset] as u64;
        } else if (GICD_ICENABLER0..=GICD_ICENABLER31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ICENABLER0) / size_of::<u32>();
            result = self.enable[register_offset] as u64;
        } else if (GICD_ISPENDR0..=GICD_ISPENDR31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ISPENDR0) / size_of::<u32>();
            result = self.pending[register_offset] as u64;
        } else if (GICD_ICPENDR0..=GICD_ICPENDR31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ICPENDR0) / size_of::<u32>();
            result = self.pending[register_offset] as u64;
        } else if (GICD_ISACTIVER0..=GICD_ISACTIVER31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ISACTIVER0) / size_of::<u32>();
            result = self.active[register_offset] as u64;
        } else if (GICD_ICACTIVER0..=GICD_ICACTIVER31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ICACTIVER0) / size_of::<u32>();
            result = self.active[register_offset] as u64;
        } else if (GICD_IPRIORITYR0..(GICD_IPRIORITYR254 + size_of::<u32>())).contains(&offset) {
            let register_offset = (offset - GICD_IPRIORITYR0) / size_of::<u32>();
            let byte_offset = (offset - GICD_IPRIORITYR0) - register_offset * size_of::<u32>();
            if access_width == 8 {
                /* Byte access */
                result = self.priority[register_offset] as u64;
                result = (result >> (byte_offset * 8)) & 0xff;
            } else if byte_offset == 0 && access_width == 32 {
                /* 32-bit access */
                result = self.priority[register_offset] as u64;
            }
        } else if (GICD_ITARGETSR0..(GICD_ITARGETSR254 + size_of::<u32>())).contains(&offset) {
            let register_offset = (offset - GICD_ITARGETSR0) / size_of::<u32>();
            let byte_offset = (offset - GICD_ITARGETSR0) - register_offset * size_of::<u32>();
            let int_id_base = register_offset * 4;
            if access_width == 8 {
                let int_id = int_id_base + byte_offset;
                if int_id < 32 {
                    /* SGI/PPI: banked per accessing pCPU -- must reflect *this* pCPU's own
                     * real GICv2 target bit, not `own_target` (which is fixed to whichever
                     * pCPU first created this VM). Getting this wrong corrupts the guest's
                     * own `gic_cpu_map`-equivalent table once more than one pCPU is running
                     * a vCPU of the same VM (true SMP), since every vCPU would otherwise
                     * believe its own target bit is the VM-creating pCPU's. */
                    result = gicv2::get_current_cpu_target() as u64;
                } else {
                    result = (self.target[register_offset] >> (byte_offset * 8)) as u64 & 0xff;
                }
            } else if byte_offset == 0 && access_width == 32 {
                if int_id_base < 32 {
                    let target = gicv2::get_current_cpu_target() as u32;
                    result = (target | (target << 8) | (target << 16) | (target << 24)) as u64;
                } else {
                    result = self.target[register_offset] as u64;
                }
            }
        } else if (GICD_ICFGR0..=GICD_ICFGR63).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ICFGR0) / size_of::<u32>();
            result = self.configuration[register_offset] as u64;
        } else if offset == GICD_PIDR2 && access_width == 32 {
            result = GIC_REVISION << 4;
        }
        Ok(result)
    }

    fn write(&mut self, offset: usize, access_width: u64, value: u64) -> Result<(), ()> {
        if offset == GICD_CTLR && access_width == 32 {
            self.ctlr = value as u32;
        } else if (GICD_IGROUPR0..=GICD_IGROUPR31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_IGROUPR0) / size_of::<u32>();
            self.group[register_offset] = value as u32;
        } else if (GICD_ISENABLER0..=GICD_ISENABLER31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ISENABLER0) / size_of::<u32>();
            self.enable[register_offset] |= value as u32;
            let mut value = value;
            for int_id in (register_offset * 32).. {
                if value == 0 {
                    break;
                }
                if (value & 1) != 0 && self.get_pending(int_id as u32) {
                    self.trigger_interrupt(int_id as u32, None);
                }
                value >>= 1;
            }
        } else if (GICD_ICENABLER0..=GICD_ICENABLER31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ICENABLER0) / size_of::<u32>();
            self.enable[register_offset] &= !(value as u32);
        } else if (GICD_ISPENDR0..=GICD_ISPENDR31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ISPENDR0) / size_of::<u32>();
            self.pending[register_offset] |= value as u32;
            let mut value = value;
            for int_id in (register_offset * size_of::<u32>() * 8).. {
                if value == 0 {
                    break;
                }
                if (value & 1) != 0 && self.get_enable(int_id as u32) {
                    self.trigger_interrupt(int_id as u32, None);
                }
                value >>= 1;
            }
        } else if (GICD_ICPENDR0..=GICD_ICPENDR31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ICPENDR0) / size_of::<u32>();
            self.pending[register_offset] &= !(value as u32);
        } else if (GICD_ISACTIVER0..=GICD_ISACTIVER31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ISACTIVER0) / size_of::<u32>();
            self.active[register_offset] |= value as u32;
        } else if (GICD_ICACTIVER0..=GICD_ICACTIVER31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ICACTIVER0) / size_of::<u32>();
            self.active[register_offset] &= !(value as u32);
        } else if (GICD_IPRIORITYR0..(GICD_IPRIORITYR254 + size_of::<u32>())).contains(&offset) {
            let register_offset = (offset - GICD_IPRIORITYR0) / size_of::<u32>();
            let byte_offset = (offset - GICD_IPRIORITYR0) - register_offset * size_of::<u32>();
            let bit_offset = byte_offset * 8;
            if access_width == 8 {
                /* Byte access */
                self.priority[register_offset] = ((self.priority[register_offset])
                    & !(0xFF << bit_offset))
                    | (value << bit_offset) as u32;
            } else if byte_offset == 0 && access_width == 32 {
                /* 32-bit access */
                self.priority[register_offset] = value as u32;
            }
        } else if (GICD_ITARGETSR0..(GICD_ITARGETSR254 + size_of::<u32>())).contains(&offset) {
            /* IDs 0-31 are Read-Only (banked registers), so ignore writes */
            let register_offset = (offset - GICD_ITARGETSR0) / size_of::<u32>();
            let byte_offset = (offset - GICD_ITARGETSR0) - register_offset * size_of::<u32>();
            let int_id_base = register_offset * 4;
            let bit_offset = byte_offset * 8;
            if int_id_base >= 32 {
                if access_width == 8 {
                    self.target[register_offset] = (self.target[register_offset]
                        & !(0xFF << bit_offset))
                        | (value << bit_offset) as u32;
                } else if byte_offset == 0 && access_width == 32 {
                    self.target[register_offset] = value as u32;
                }
            }
        } else if (GICD_ICFGR0..=GICD_ICFGR63).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ICFGR0) / size_of::<u32>();
            self.configuration[register_offset] = value as u32;
        } else if offset == GICD_SGIR && access_width == 32 {
            let int_id = (value as u32) & 0xF;
            /* GICD_SGIR[25:24] = TargetListFilter, GICD_SGIR[23:16] = CPUTargetList (a raw
             * 8-bit GICv2 CPU target bitmask -- since each vCPU of this VM runs 1:1 on a real
             * physical core, this bitmask can be used directly as the physical pCPU target
             * bitmask, with no virtual-to-physical translation needed). */
            const ALL_PCPU_TARGETS: u8 = 0x0F; /* This platform supports up to 4 pCPUs. */
            let target_list_filter = (value >> 24) & 0b11;
            let targets = match target_list_filter {
                0b00 => ((value >> 16) & 0xFF) as u8,
                0b01 => ALL_PCPU_TARGETS & !gicv2::get_current_cpu_target(),
                _ => gicv2::get_current_cpu_target(),
            };
            self.trigger_interrupt_to(int_id, None, targets);
        }
        Ok(())
    }
}
