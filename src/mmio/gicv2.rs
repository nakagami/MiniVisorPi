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
 * registers in the same MMIO region as the Distributor, so SPI and SGI/PPI are handled
 * together in a single struct. The SGI/PPI half of those registers is *banked per
 * accessing CPU interface* by real hardware, though, so it is kept in a separate
 * per-pCPU `banked` array here rather than in the shared, SPI-wide arrays (see
 * [`BankedRegisters`]).
 */
pub struct GicDistributorMmio {
    ctlr: u32,
    /* The arrays below hold only the *shared* (SPI, INTID >= 32) half of each register
     * array; register indices covering INTID 0-31 are routed to `banked` instead by
     * `group_register`/`enable_register`/etc., and their entries here are unused. */
    group: [u32; 32],
    enable: [u32; 32],
    pending: [u32; 32],
    active: [u32; 32],
    priority: [u32; 255],
    configuration: [u32; 64],
    target: [u32; 255],
    /// Per-pCPU banked copies of the SGI/PPI (INTID 0-31) registers.
    banked: [BankedRegisters; MAX_CPU_INTERFACES],
    /// GICv2 target bit of the physical CPU running this VM
    own_target: u8,
    to_inject_interrupt: LinkedList<(u8, u32)>,
}

/// Number of physical GICv2 CPU interfaces a target bitmask can address.
const MAX_CPU_INTERFACES: usize = u8::BITS as usize;
/// INTIDs 0-31 (SGI 0-15 + PPI 16-31) are banked per CPU interface.
const BANKED_INT_ID_COUNT: usize = 32;

/// The SGI/PPI (INTID 0-31) half of the Distributor's register arrays, which GICv2 banks
/// per accessing CPU interface: every CPU sees and owns its *own* private copy at the very
/// same MMIO offsets.
///
/// Emulating these as shared state is fatal once a single VM has several vCPUs running
/// concurrently on separate pCPUs (true SMP). Linux's `gic_cpu_init` runs on *every* CPU as
/// it comes online and starts by writing `GICD_ICENABLER0 = 0xFFFF0000` to disable all of
/// *its own* PPIs; with shared state that write also disabled the already-running CPUs'
/// Generic Timer PPI. Any timer interrupt that then fired on those CPUs was silently
/// dropped by `trigger_interrupt_to` while the hypervisor's IRQ handler deliberately left
/// the *physical* interrupt active for the vGIC to deactivate later, so it stayed active
/// forever and that pCPU's timer never fired again -- the guest lost its scheduler tick
/// permanently and hung with an RCU stall before reaching login.
#[derive(Clone, Copy)]
struct BankedRegisters {
    group: u32,
    enable: u32,
    pending: u32,
    active: u32,
    /// 8 bits per INTID, so 4 INTIDs per 32-bit register.
    priority: [u32; BANKED_INT_ID_COUNT / 4],
    /// 2 bits per INTID, so 16 INTIDs per 32-bit register.
    configuration: [u32; BANKED_INT_ID_COUNT / 16],
}

impl BankedRegisters {
    const fn new() -> Self {
        Self {
            group: 0,
            enable: 0,
            pending: 0,
            active: 0,
            priority: [0; BANKED_INT_ID_COUNT / 4],
            configuration: [0; BANKED_INT_ID_COUNT / 16],
        }
    }
}

/// Maps a single-bit GICv2 CPU target mask to an index into [`GicDistributorMmio::banked`].
fn target_to_cpu_index(target: u8) -> usize {
    (target.trailing_zeros() as usize).min(MAX_CPU_INTERFACES - 1)
}

/// Index into [`GicDistributorMmio::banked`] of the pCPU executing this code. Since every
/// vCPU of a VM runs 1:1 on its own physical core, "the pCPU performing this access" is
/// also "the vCPU whose banked registers this access refers to".
fn current_cpu_index() -> usize {
    target_to_cpu_index(gicv2::get_current_cpu_target())
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
            banked: [BankedRegisters::new(); MAX_CPU_INTERFACES],
            own_target,
            to_inject_interrupt: LinkedList::new(),
        }
    }

    /* The `*_register` helpers below are the single place that decides whether a given
     * register index refers to the banked SGI/PPI (INTID 0-31) range -- which must come
     * from the accessing/target pCPU's own bank -- or to the shared SPI range. */

    fn group_register(&mut self, cpu: usize, index: usize) -> &mut u32 {
        if index == 0 {
            &mut self.banked[cpu].group
        } else {
            &mut self.group[index]
        }
    }

    fn enable_register(&mut self, cpu: usize, index: usize) -> &mut u32 {
        if index == 0 {
            &mut self.banked[cpu].enable
        } else {
            &mut self.enable[index]
        }
    }

    fn pending_register(&mut self, cpu: usize, index: usize) -> &mut u32 {
        if index == 0 {
            &mut self.banked[cpu].pending
        } else {
            &mut self.pending[index]
        }
    }

    fn active_register(&mut self, cpu: usize, index: usize) -> &mut u32 {
        if index == 0 {
            &mut self.banked[cpu].active
        } else {
            &mut self.active[index]
        }
    }

    fn priority_register(&mut self, cpu: usize, index: usize) -> &mut u32 {
        if index < BANKED_INT_ID_COUNT / 4 {
            &mut self.banked[cpu].priority[index]
        } else {
            &mut self.priority[index]
        }
    }

    fn configuration_register(&mut self, cpu: usize, index: usize) -> &mut u32 {
        if index < BANKED_INT_ID_COUNT / 16 {
            &mut self.banked[cpu].configuration[index]
        } else {
            &mut self.configuration[index]
        }
    }

    fn get_group(&mut self, cpu: usize, int_id: u32) -> u32 {
        let int_id = int_id as usize;
        const DATA_PER_REG: usize = u32::BITS as usize;
        let register = int_id / DATA_PER_REG;
        let offset = int_id % DATA_PER_REG;
        (*self.group_register(cpu, register) >> offset) & 0b1
    }

    fn get_enable(&mut self, cpu: usize, int_id: u32) -> bool {
        let int_id = int_id as usize;
        const DATA_PER_REG: usize = u32::BITS as usize;
        let register = int_id / DATA_PER_REG;
        let offset = int_id % DATA_PER_REG;
        ((*self.enable_register(cpu, register) >> offset) & 1) != 0
    }

    fn get_pending(&mut self, cpu: usize, int_id: u32) -> bool {
        let int_id = int_id as usize;
        const DATA_PER_REG: usize = u32::BITS as usize;
        let register = int_id / DATA_PER_REG;
        let offset = int_id % DATA_PER_REG;
        ((*self.pending_register(cpu, register) >> offset) & 1) != 0
    }

    /// Sets/clears an INTID's pending bit in `cpu`'s view of the Distributor.
    fn change_pending_status_of(&mut self, cpu: usize, int_id: u32, is_pending: bool) {
        let int_id = int_id as usize;
        const DATA_PER_REG: usize = u32::BITS as usize;
        let register = int_id / DATA_PER_REG;
        let offset = int_id % DATA_PER_REG;
        let register = self.pending_register(cpu, register);
        if is_pending {
            *register |= 1 << offset;
        } else {
            *register &= !(1 << offset);
        }
    }

    pub fn change_pending_status(&mut self, int_id: u32, is_pending: bool) {
        self.change_pending_status_of(current_cpu_index(), int_id, is_pending);
    }

    pub fn change_active_status(&mut self, int_id: u32, is_active: bool) {
        let cpu = current_cpu_index();
        let int_id = int_id as usize;
        const DATA_PER_REG: usize = u32::BITS as usize;
        let register = int_id / DATA_PER_REG;
        let offset = int_id % DATA_PER_REG;
        let register = self.active_register(cpu, register);
        if is_active {
            *register |= 1 << offset;
        } else {
            *register &= !(1 << offset);
        }
    }

    fn get_priority(&mut self, cpu: usize, int_id: u32) -> u32 {
        let int_id = int_id as usize;
        const DATA_PER_REG: usize = u32::BITS as usize / 8;
        let register = int_id / DATA_PER_REG;
        let offset = (int_id % DATA_PER_REG) * 8;
        (*self.priority_register(cpu, register) >> offset) & 0xFF
    }

    /// Re-delivers an already-pending INTID after the guest enabled/re-pended it via MMIO.
    /// A banked INTID (SGI/PPI, 0-31) belongs to the pCPU that performed the write, while an
    /// SPI belongs to `own_target`.
    fn retrigger(&mut self, int_id: u32) -> bool {
        let targets = if (int_id as usize) < BANKED_INT_ID_COUNT {
            gicv2::get_current_cpu_target()
        } else {
            self.own_target
        };
        self.trigger_interrupt_to(int_id, None, targets)
    }

    /// Returns `true` if the interrupt was actually injected into (or queued for) at least
    /// one target pCPU. Callers that passed a `physical_int_id` (i.e. an LR with the HW bit,
    /// whose deactivation the guest is expected to perform) **must** deactivate the physical
    /// interrupt themselves when this returns `false`: otherwise it stays active in the
    /// physical GIC forever and that pCPU never receives that interrupt again.
    pub fn trigger_interrupt(&mut self, int_id: u32, physical_int_id: Option<u32>) -> bool {
        self.trigger_interrupt_to(int_id, physical_int_id, self.own_target)
    }

    /// Same as [`trigger_interrupt`](Self::trigger_interrupt), but delivers to an explicit
    /// set of target pCPUs (a GICv2 8-bit CPU target bitmask) instead of always `own_target`.
    /// Used for SGIs (`GICD_SGIR`), whose destination is chosen per-write by the sending
    /// vCPU rather than being a fixed, interrupt-ID-owned target -- essential once a single
    /// VM can have multiple concurrently-running vCPUs (true guest SMP), since an SGI/IPI
    /// sent by one vCPU must reach whichever *other* vCPU(s) it targets, not just whichever
    /// pCPU originally created the VM.
    pub fn trigger_interrupt_to(
        &mut self,
        int_id: u32,
        physical_int_id: Option<u32>,
        targets: u8,
    ) -> bool {
        /* In the single security state with security_extn=false (secure=off), only bit0
         * (Enable) of GICD_CTLR is meaningful, and Linux's GICv2 driver also only writes
         * bit0. Since bit1 (Grp1 Enable) is unused, treat it as enabled if either bit is set. */
        let distributor_enabled =
            (self.ctlr & (GICD_CTLR_ENABLE_GRP0 | GICD_CTLR_ENABLE_GRP1)) != 0;
        let current_target = gicv2::get_current_cpu_target();
        let mut injected = false;

        for bit in 0..u8::BITS {
            let target = 1u8 << bit;
            if (targets & target) == 0 {
                continue;
            }
            /* Enable/group/priority/pending of INTID 0-31 are banked per CPU interface,
             * so they must be evaluated against each *target's* own bank, not the
             * sending CPU's. */
            let cpu = target_to_cpu_index(target);
            self.change_pending_status_of(cpu, int_id, true);
            if !distributor_enabled || !self.get_enable(cpu, int_id) {
                continue;
            }
            let group = self.get_group(cpu, int_id);
            let priority = self.get_priority(cpu, int_id);
            let list_entry =
                vgic::create_list_register_entry(int_id, group, priority, physical_int_id);

            if target == current_target {
                /* Same pCPU: inject directly into this pCPU's own List Registers. */
                injected |= vgic::add_virtual_interrupt(list_entry);
            } else {
                /* A different pCPU (either a different VM, sharing this pCPU with the
                 * cooperative scheduler, or a different vCPU of *this* same VM, running
                 * concurrently on another physical core): queue the entry tagged with
                 * its intended destination and prod that pCPU with a physical SGI so it
                 * services just its own share of the queue. */
                self.to_inject_interrupt.push_back((target, list_entry));
                gicv2::send_sgi(target, INJECT_INTERRUPT_INT_ID);
                injected = true;
            }
        }
        injected
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
            let _ = vgic::add_virtual_interrupt(entry);
        } else {
            remaining.push_back((target, entry));
        }
    }
    distributor.to_inject_interrupt = remaining;
}

impl MmioHandler for GicDistributorMmio {
    fn read(&mut self, offset: usize, access_width: u64) -> Result<u64, ()> {
        let mut result = 0u64;
        /* INTID 0-31 live in the accessing pCPU's own bank; see `*_register()`. */
        let cpu = current_cpu_index();
        if offset == GICD_CTLR && access_width == 32 {
            result = self.ctlr as u64;
        } else if offset == GICD_TYPER && access_width == 32 {
            result = GICD_TYPER_VALUE as u64;
        } else if (GICD_IGROUPR0..=GICD_IGROUPR31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_IGROUPR0) / size_of::<u32>();
            result = *self.group_register(cpu, register_offset) as u64;
        } else if (GICD_ISENABLER0..=GICD_ISENABLER31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ISENABLER0) / size_of::<u32>();
            result = *self.enable_register(cpu, register_offset) as u64;
        } else if (GICD_ICENABLER0..=GICD_ICENABLER31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ICENABLER0) / size_of::<u32>();
            result = *self.enable_register(cpu, register_offset) as u64;
        } else if (GICD_ISPENDR0..=GICD_ISPENDR31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ISPENDR0) / size_of::<u32>();
            result = *self.pending_register(cpu, register_offset) as u64;
        } else if (GICD_ICPENDR0..=GICD_ICPENDR31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ICPENDR0) / size_of::<u32>();
            result = *self.pending_register(cpu, register_offset) as u64;
        } else if (GICD_ISACTIVER0..=GICD_ISACTIVER31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ISACTIVER0) / size_of::<u32>();
            result = *self.active_register(cpu, register_offset) as u64;
        } else if (GICD_ICACTIVER0..=GICD_ICACTIVER31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ICACTIVER0) / size_of::<u32>();
            result = *self.active_register(cpu, register_offset) as u64;
        } else if (GICD_IPRIORITYR0..(GICD_IPRIORITYR254 + size_of::<u32>())).contains(&offset) {
            let register_offset = (offset - GICD_IPRIORITYR0) / size_of::<u32>();
            let byte_offset = (offset - GICD_IPRIORITYR0) - register_offset * size_of::<u32>();
            if access_width == 8 {
                /* Byte access */
                result = *self.priority_register(cpu, register_offset) as u64;
                result = (result >> (byte_offset * 8)) & 0xff;
            } else if byte_offset == 0 && access_width == 32 {
                /* 32-bit access */
                result = *self.priority_register(cpu, register_offset) as u64;
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
            result = *self.configuration_register(cpu, register_offset) as u64;
        } else if offset == GICD_PIDR2 && access_width == 32 {
            result = GIC_REVISION << 4;
        }
        Ok(result)
    }

    fn write(&mut self, offset: usize, access_width: u64, value: u64) -> Result<(), ()> {
        /* INTID 0-31 live in the accessing pCPU's own bank; see `*_register()`. */
        let cpu = current_cpu_index();
        if offset == GICD_CTLR && access_width == 32 {
            self.ctlr = value as u32;
        } else if (GICD_IGROUPR0..=GICD_IGROUPR31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_IGROUPR0) / size_of::<u32>();
            *self.group_register(cpu, register_offset) = value as u32;
        } else if (GICD_ISENABLER0..=GICD_ISENABLER31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ISENABLER0) / size_of::<u32>();
            *self.enable_register(cpu, register_offset) |= value as u32;
            let mut value = value;
            for int_id in (register_offset * 32).. {
                if value == 0 {
                    break;
                }
                if (value & 1) != 0 && self.get_pending(cpu, int_id as u32) {
                    let _ = self.retrigger(int_id as u32);
                }
                value >>= 1;
            }
        } else if (GICD_ICENABLER0..=GICD_ICENABLER31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ICENABLER0) / size_of::<u32>();
            *self.enable_register(cpu, register_offset) &= !(value as u32);
        } else if (GICD_ISPENDR0..=GICD_ISPENDR31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ISPENDR0) / size_of::<u32>();
            *self.pending_register(cpu, register_offset) |= value as u32;
            let mut value = value;
            for int_id in (register_offset * size_of::<u32>() * 8).. {
                if value == 0 {
                    break;
                }
                if (value & 1) != 0 && self.get_enable(cpu, int_id as u32) {
                    let _ = self.retrigger(int_id as u32);
                }
                value >>= 1;
            }
        } else if (GICD_ICPENDR0..=GICD_ICPENDR31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ICPENDR0) / size_of::<u32>();
            *self.pending_register(cpu, register_offset) &= !(value as u32);
        } else if (GICD_ISACTIVER0..=GICD_ISACTIVER31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ISACTIVER0) / size_of::<u32>();
            *self.active_register(cpu, register_offset) |= value as u32;
        } else if (GICD_ICACTIVER0..=GICD_ICACTIVER31).contains(&offset) && access_width == 32 {
            let register_offset = (offset - GICD_ICACTIVER0) / size_of::<u32>();
            *self.active_register(cpu, register_offset) &= !(value as u32);
        } else if (GICD_IPRIORITYR0..(GICD_IPRIORITYR254 + size_of::<u32>())).contains(&offset) {
            let register_offset = (offset - GICD_IPRIORITYR0) / size_of::<u32>();
            let byte_offset = (offset - GICD_IPRIORITYR0) - register_offset * size_of::<u32>();
            let bit_offset = byte_offset * 8;
            if access_width == 8 {
                /* Byte access */
                let register = self.priority_register(cpu, register_offset);
                *register = (*register & !(0xFF << bit_offset)) | (value << bit_offset) as u32;
            } else if byte_offset == 0 && access_width == 32 {
                /* 32-bit access */
                *self.priority_register(cpu, register_offset) = value as u32;
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
            *self.configuration_register(cpu, register_offset) = value as u32;
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
            let _ = self.trigger_interrupt_to(int_id, None, targets);
        }
        Ok(())
    }
}
