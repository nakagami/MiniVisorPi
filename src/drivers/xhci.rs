//!
//! USB xHCI host-controller driver
//!
//! Implements the minimum xHCI bring-up needed for Raspberry Pi 4's external
//! VL805 controller when firmware/U-Boot has already trained PCIe and assigned
//! PCI BARs. The VL805 is one unified xHCI controller for both USB 2.0
//! (Low/Full/High speed) and USB 3.0 (SuperSpeed) devices, so this single
//! driver handles both through the same command-ring/TRB/context model with
//! speed-specific branches where the xHCI and USB specs require them.
//!
//! SuperSpeed-specific handling currently includes the fixed 512-byte default
//! control endpoint size on EP0 and parsing the SuperSpeed Endpoint Companion
//! Descriptor so bulk endpoint contexts program the correct Max Burst field.
//! System Exit Latency / U1/U2 power-management negotiation is intentionally
//! left unimplemented for now; many minimal embedded host stacks function
//! correctly without it, at the cost of not using those lower-power link
//! states. This is still a from-scratch bring-up driver that has **not** been
//! validated on physical hardware yet: register layouts, ring/context setup,
//! and enumeration flow follow the Raspberry Pi 4's own U-Boot xHCI stack, but
//! real-hardware testing is still required before relying on it.
//!

use crate::asm;
use core::cmp::min;
use core::mem::size_of;
use core::ptr::{copy_nonoverlapping, read_volatile, write_bytes, write_volatile};
use core::sync::atomic::{AtomicU64, Ordering};

/// CPU-physical → PCIe-bus address offset for downstream DMA masters.
///
/// On the BCM2711 the PCIe inbound window is not identity-mapped: the VL805
/// xHCI controller sees system RAM at `cpu_phys + DMA_OFFSET` (observed
/// 0x4_00000000). Every address handed to the controller — pointer registers,
/// the DCBAA/ERST/scratchpad structures, TRB ring link pointers, endpoint
/// context dequeue pointers and transfer data buffers — must therefore be
/// translated with [`to_bus`], while CPU-side accesses keep using the raw
/// physical address. The value is read from the PCIe root complex at init.
static DMA_OFFSET: AtomicU64 = AtomicU64::new(0);

/// Translates a CPU physical address into the PCIe bus address the xHCI
/// controller must be programmed with. See [`DMA_OFFSET`].
fn to_bus(cpu_addr: usize) -> u64 {
    cpu_addr as u64 + DMA_OFFSET.load(Ordering::Relaxed)
}

const MAX_HC_SLOTS: usize = 256;
const MAX_EP_CTX_NUM: usize = 31;
const TRBS_PER_SEGMENT: usize = 64;
const LINK_TRB_INDEX: usize = TRBS_PER_SEGMENT - 1;
const POLL_TIMEOUT_US: u64 = 5_000_000;
const RESET_TIMEOUT_US: u64 = 250_000;
const HALT_TIMEOUT_US: u64 = 16_000;
const HUB_SHORT_RESET_DELAY_US: u64 = 20_000;
const HUB_LONG_RESET_DELAY_US: u64 = 200_000;

const USBCMD_RUN: u32 = 1 << 0;
const USBCMD_RESET: u32 = 1 << 1;
const USBSTS_HALT: u32 = 1 << 0;
const USBSTS_HSE: u32 = 1 << 2;
const USBSTS_CNR: u32 = 1 << 11;

const CONFIG_MAX_SLOTS_MASK: u32 = 0xFF;
const HCS_MAX_SLOTS_MASK: u32 = 0xFF;
const HCS_MAX_PORTS_SHIFT: u32 = 24;
const HCS_MAX_PORTS_MASK: u32 = 0xFF;
const HCS_MAX_SCRATCHPAD_HI_SHIFT: u32 = 16;
const HCS_MAX_SCRATCHPAD_LO_SHIFT: u32 = 27;
const HCC_64BYTE_CONTEXT: u32 = 1 << 2;
const HCC_PPC: u32 = 1 << 3;

const DBOFF_MASK: u32 = !0x3;
const RTSOFF_MASK: u32 = !0x1F;
const CMD_RING_RSVD_BITS: u64 = 0x3F;
const ERST_PTR_MASK: u64 = 0xF;
const ERST_EHB: u64 = 1 << 3;

const PORT_CONNECT: u32 = 1 << 0;
const PORT_PE: u32 = 1 << 1;
const PORT_RESET: u32 = 1 << 4;
const PORT_POWER: u32 = 1 << 9;
const DEV_SPEED_MASK: u32 = 0xF << 10;
const XDEV_FS: u32 = 0x1 << 10;
const XDEV_LS: u32 = 0x2 << 10;
const XDEV_HS: u32 = 0x3 << 10;
const XDEV_SS: u32 = 0x4 << 10;
const PORT_CSC: u32 = 1 << 17;
const PORT_PEC: u32 = 1 << 18;
const PORT_WRC: u32 = 1 << 19;
const PORT_OCC: u32 = 1 << 20;
const PORT_RC: u32 = 1 << 21;
const PORT_PLC: u32 = 1 << 22;
const PORT_CEC: u32 = 1 << 23;

const XHCI_PORT_RO: u32 = (1 << 0) | (1 << 3) | (0xF << 10) | (1 << 30);
const XHCI_PORT_RWS: u32 = (0xF << 5) | (1 << 9) | (0x3 << 14) | (0x7 << 25);

const SLOT_SPEED_FS: u32 = XDEV_FS << 10;
const SLOT_SPEED_LS: u32 = XDEV_LS << 10;
const SLOT_SPEED_HS: u32 = XDEV_HS << 10;
const SLOT_SPEED_SS: u32 = XDEV_SS << 10;
const DEV_MTT: u32 = 1 << 25;
const DEV_HUB: u32 = 1 << 26;
const LAST_CTX_SHIFT: u32 = 27;
const SLOT_FLAG: u32 = 1 << 0;
const EP0_FLAG: u32 = 1 << 1;
const ROOT_HUB_PORT_SHIFT: u32 = 16;
const MAX_PORTS_SHIFT: u32 = 24;

const EP_TYPE_SHIFT: u32 = 3;
const CTRL_EP: u32 = 4;
const BULK_OUT_EP: u32 = 2;
const BULK_IN_EP: u32 = 6;
const MAX_BURST_SHIFT: u32 = 8;
const MAX_PACKET_SHIFT: u32 = 16;
const ERROR_COUNT_SHIFT: u32 = 1;
const EP_AVG_TRB_LENGTH_MASK: u32 = 0xFFFF;

const TRB_CYCLE: u32 = 1 << 0;
const TRB_ISP: u32 = 1 << 2;
const TRB_CHAIN: u32 = 1 << 4;
const TRB_IOC: u32 = 1 << 5;
const TRB_IDT: u32 = 1 << 6;
const TRB_DIR_IN: u32 = 1 << 16;
const TRB_TYPE_SHIFT: u32 = 10;
const TRB_TYPE_MASK: u32 = 0xFC00;
const LINK_TOGGLE: u32 = 1 << 1;

const TRB_ENABLE_SLOT: u32 = 9;
const TRB_ADDR_DEV: u32 = 11;
const TRB_CONFIG_EP: u32 = 12;
const TRB_TRANSFER_EVENT: u32 = 32;
const TRB_COMPLETION_EVENT: u32 = 33;
const TRB_NORMAL: u32 = 1;
const TRB_SETUP: u32 = 2;
const TRB_DATA: u32 = 3;
const TRB_STATUS: u32 = 4;
const TRB_LINK: u32 = 6;

const COMP_SUCCESS: u32 = 1;
const COMP_SHORT_TX: u32 = 13;

const TT_PORT_SHIFT: u32 = 8;
const TT_THINK_TIME_SHIFT: u32 = 16;

const USB_REQ_GET_STATUS: u8 = 0;
const USB_REQ_CLEAR_FEATURE: u8 = 1;
const USB_REQ_SET_FEATURE: u8 = 3;
const USB_REQ_GET_DESCRIPTOR: u8 = 6;
const USB_REQ_SET_CONFIGURATION: u8 = 9;
const USB_REQ_SET_INTERFACE: u8 = 11;
const USB_DIR_IN: u8 = 0x80;
const USB_RT_HUB: u8 = 0x20;
const USB_RT_PORT: u8 = 0x23;
const USB_RECIP_INTERFACE: u8 = 0x01;
const USB_CLASS_HUB: u8 = 0x09;
const USB_DT_DEVICE: u8 = 1;
const USB_DT_CONFIG: u8 = 2;
const USB_DT_INTERFACE: u8 = 4;
const USB_DT_ENDPOINT: u8 = 5;
const USB_DT_HUB: u8 = 0x29;
const USB_DT_SS_HUB: u8 = 0x2A;
const USB_DT_SS_EP_COMPANION: u8 = 0x30;
const USB_ENDPOINT_XFER_BULK: u8 = 2;
const USB_ENDPOINT_DIR_MASK: u8 = 0x80;
const USB_PORT_FEAT_RESET: u16 = 4;
const USB_PORT_FEAT_POWER: u16 = 8;
const USB_PORT_FEAT_C_RESET: u16 = 20;
const USB_PORT_STAT_CONNECTION: u16 = 0x0001;
const USB_PORT_STAT_ENABLE: u16 = 0x0002;
const USB_PORT_STAT_RESET: u16 = 0x0010;
const USB_PORT_STAT_LOW_SPEED: u16 = 0x0200;
const USB_PORT_STAT_HIGH_SPEED: u16 = 0x0400;
const USB_PORT_STAT_SUPER_SPEED: u16 = 0x0600;
const USB_PORT_STAT_SPEED_MASK: u16 = USB_PORT_STAT_LOW_SPEED | USB_PORT_STAT_HIGH_SPEED;
const USB_PORT_STAT_C_RESET: u16 = 0x0010;
const HUB_CHAR_LPSM: u16 = 0x0003;
const HUB_CHAR_INDV_PORT_LPSM: u16 = 0x0001;
const HUB_CHAR_TTTT: u16 = 0x0060;
const HUB_CHAR_TTTT_SHIFT: u32 = 5;
const USB_HUB_PR_HS_MULTI_TT: u8 = 2;

#[derive(Clone, Copy, Eq, PartialEq)]
pub enum UsbSpeed {
    Low,
    Full,
    High,
    Super,
}

#[derive(Clone, Copy)]
pub struct SetupPacket {
    pub request_type: u8,
    pub request: u8,
    pub value: u16,
    pub index: u16,
    pub length: u16,
}

pub struct XhciMassStorageDevice {
    controller: Xhci,
    device: XhciDevice,
    bulk_in_ep: u8,
    bulk_out_ep: u8,
    bulk_in_index: usize,
    bulk_out_index: usize,
    max_packet_in: u16,
    max_packet_out: u16,
}

pub struct Xhci {
    operational_base: usize,
    doorbell_base: usize,
    runtime_base: usize,
    hcsparams1: u32,
    hcsparams2: u32,
    hccparams: u32,
    hci_version: u16,
    page_size: usize,
    dcbaa: DmaRegion,
    command_ring: TrbRing,
    event_ring: EventRing,
    erst: DmaRegion,
    scratchpad_array: Option<DmaRegion>,
    scratchpad_buffers: Option<DmaRegion>,
}

struct XhciDevice {
    slot_id: u8,
    root_hub_port: u8,
    speed: UsbSpeed,
    route_string: u32,
    tt_hub_slot_id: u8,
    tt_port_number: u8,
    multi_tt: bool,
    ep0_max_packet: u16,
    out_ctx: DmaRegion,
    in_ctx: DmaRegion,
    ep0_ring: TrbRing,
    bulk_out_ring: Option<TrbRing>,
    bulk_in_ring: Option<TrbRing>,
}

struct DmaRegion {
    address: usize,
    size: usize,
    pages: usize,
}

struct TrbRing {
    region: DmaRegion,
    enqueue_index: usize,
    cycle_state: u32,
}

struct EventRing {
    region: DmaRegion,
    dequeue_index: usize,
    cycle_state: u32,
}

struct BounceBuffer {
    region: DmaRegion,
}

struct MassStorageConfiguration {
    bulk_in_ep: u8,
    bulk_out_ep: u8,
    max_packet_in: u16,
    max_packet_out: u16,
    max_burst_in: u8,
    max_burst_out: u8,
    max_ep_index: usize,
}

/// A fully configured mass-storage device found during the root-port scan,
/// carried out of the (borrowing) probe helpers so that the controller itself
/// can be moved into [`XhciMassStorageDevice`] afterwards.
struct ProbedMassStorage {
    device: XhciDevice,
    config: MassStorageConfiguration,
    bulk_out_index: usize,
    bulk_in_index: usize,
}

struct HubDescriptor {
    num_ports: u8,
    power_on_to_good_ms: u16,
    tt_think_time: u8,
    multi_tt: bool,
}

#[derive(Clone, Copy)]
struct HubPortStatus {
    status: u16,
    change: u16,
}

#[repr(C)]
struct InputControlContext {
    drop_flags: u32,
    add_flags: u32,
    reserved: [u32; 6],
}

#[repr(C)]
struct SlotContext {
    dev_info: u32,
    dev_info2: u32,
    tt_info: u32,
    dev_state: u32,
    reserved: [u32; 4],
}

#[repr(C)]
struct EndpointContext {
    ep_info: u32,
    ep_info2: u32,
    deq: u64,
    tx_info: u32,
    reserved: [u32; 3],
}

#[repr(C)]
struct ErstEntry {
    seg_addr: u64,
    seg_size: u32,
    reserved: u32,
}

impl XhciMassStorageDevice {
    pub fn bulk_in_endpoint(&self) -> u8 {
        self.bulk_in_ep
    }

    pub fn bulk_out_endpoint(&self) -> u8 {
        self.bulk_out_ep
    }

    pub fn bulk_transfer(
        &mut self,
        endpoint_address: u8,
        buffer_address: usize,
        length: usize,
    ) -> Result<usize, ()> {
        if endpoint_address == self.bulk_in_ep {
            let ring = self.device.bulk_in_ring.as_mut().ok_or(())?;
            self.controller.submit_bulk_transfer(
                self.device.slot_id,
                self.bulk_in_index,
                ring,
                buffer_address,
                length,
                self.max_packet_in,
                true,
            )
        } else if endpoint_address == self.bulk_out_ep {
            let ring = self.device.bulk_out_ring.as_mut().ok_or(())?;
            self.controller.submit_bulk_transfer(
                self.device.slot_id,
                self.bulk_out_index,
                ring,
                buffer_address,
                length,
                self.max_packet_out,
                false,
            )
        } else {
            Err(())
        }
    }
}

impl Xhci {
    pub fn new(base_address: usize, dma_offset: u64) -> Result<Self, ()> {
        DMA_OFFSET.store(dma_offset, Ordering::Relaxed);
        let capbase = Self::read32(base_address);
        let hcsparams1 = Self::read32(base_address + 0x04);
        let hcsparams2 = Self::read32(base_address + 0x08);
        let hccparams = Self::read32(base_address + 0x10);
        let capability_length = (capbase & 0xFF) as usize;
        let operational_base = base_address + capability_length;
        let doorbell_base =
            base_address + ((Self::read32(base_address + 0x14) & DBOFF_MASK) as usize);
        let runtime_base =
            base_address + ((Self::read32(base_address + 0x18) & RTSOFF_MASK) as usize);
        let hci_version = (capbase >> 16) as u16;

        let page_size_bits = Self::read32(operational_base + 0x08) & 0xFFFF;
        let mut page_shift = None;
        for i in 0..16 {
            if (page_size_bits & (1 << i)) != 0 {
                page_shift = Some(12 + i);
                break;
            }
        }
        let Some(page_shift) = page_shift else {
            println!("xHCI: controller advertises no supported page size");
            return Err(());
        };
        let page_size = 1usize << page_shift;

        let dcbaa = DmaRegion::new(size_of::<u64>() * MAX_HC_SLOTS, 12).map_err(|_| {
            println!("xHCI: failed to allocate the Device Context Base Address Array");
        })?;
        let command_ring = TrbRing::new(true).map_err(|_| {
            println!("xHCI: failed to allocate the command ring");
        })?;
        let event_ring = EventRing::new().map_err(|_| {
            println!("xHCI: failed to allocate the event ring");
        })?;
        let erst = DmaRegion::new(size_of::<ErstEntry>(), 12).map_err(|_| {
            println!("xHCI: failed to allocate the Event Ring Segment Table");
        })?;

        let mut controller = Self {
            operational_base,
            doorbell_base,
            runtime_base,
            hcsparams1,
            hcsparams2,
            hccparams,
            hci_version,
            page_size,
            dcbaa,
            command_ring,
            event_ring,
            erst,
            scratchpad_array: None,
            scratchpad_buffers: None,
        };

        controller.bring_up().map_err(|_| {
            println!("xHCI: controller bring-up failed after retries");
        })?;
        println!(
            "USB XHCI {}.{:02X}",
            controller.hci_version >> 8,
            controller.hci_version & 0xFF
        );
        Ok(controller)
    }

    /// Runs the reset/program/start sequence, retrying a few times on
    /// failure. Real Raspberry Pi 4 hardware has been observed to
    /// occasionally raise a Host System Error (USBSTS.HSE) or fail to leave
    /// the halted state on the very first attempt, likely a marginal timing
    /// issue in the VL805/PCIe bring-up rather than a deterministic driver
    /// bug (the exact same code path succeeds on other boots without
    /// change). A full HC reset clears that error state, so simply retrying
    /// the whole sequence a few times is a pragmatic way to ride out that
    /// flakiness instead of failing outright on the first bad attempt.
    fn bring_up(&mut self) -> Result<(), ()> {
        const MAX_ATTEMPTS: u32 = 4;
        for attempt in 1..=MAX_ATTEMPTS {
            /* Clear any stale RW1C status bits (Host System Error, Event
             * Interrupt, Port Change Detect, Save/Restore Error) left over
             * from a previous failed attempt before trying again. */
            self.write_operational32(0x04, self.read_operational32(0x04));

            let result = (|| -> Result<(), ()> {
                self.reset_controller()
                    .map_err(|_| println!("xHCI:   reset_controller failed"))?;
                self.program_max_slots();
                self.initialize_memory_structures()
                    .map_err(|_| println!("xHCI:   initialize_memory_structures failed"))?;
                self.start_controller()
                    .map_err(|_| println!("xHCI:   start_controller failed"))
            })();

            match result {
                Ok(()) => return Ok(()),
                Err(()) => {
                    if attempt == MAX_ATTEMPTS {
                        return Err(());
                    }
                    println!(
                        "xHCI: bring-up attempt {attempt}/{MAX_ATTEMPTS} failed (USBSTS={:#X}), retrying",
                        self.read_operational32(0x04)
                    );
                    Self::delay_ms(50);
                }
            }
        }
        Err(())
    }

    pub fn probe_mass_storage(mut self) -> Result<XhciMassStorageDevice, ()> {
        let max_ports = ((self.hcsparams1 >> HCS_MAX_PORTS_SHIFT) & HCS_MAX_PORTS_MASK) as usize;
        let power_control = (self.hccparams & HCC_PPC) != 0;
        println!(
            "xHCI: scanning {} root port(s) (port power control: {})",
            max_ports, power_control
        );

        /*
         * Every root port must be tried, not just the first one that reports a
         * connection: the Raspberry Pi 4's VL805 exposes its onboard VIA Labs
         * hub twice -- as a USB 2.0 hub on a High-Speed root port and as a USB
         * 3.0 hub on a SuperSpeed root port -- and each physical socket is
         * routed to whichever of the two matches the speed the attached device
         * negotiated. A SuperSpeed stick therefore leaves all downstream ports
         * of the High-Speed hub (which sits on the lowest-numbered root port,
         * and so is scanned first) reporting "no connection", so stopping
         * there would never reach the hub that actually has the device.
         */
        let mut found = None;
        for port in 1..=max_ports {
            let port = port as u8;
            let Some(speed) = self.reset_root_port(port, power_control) else {
                continue;
            };
            match self.probe_root_port_device(port, speed) {
                Ok(probed) => {
                    found = Some(probed);
                    break;
                }
                Err(()) => println!(
                    "xHCI: no usable mass-storage device behind root port {port}, \
                     continuing the scan"
                ),
            }
        }
        let Some(probed) = found else {
            println!("xHCI: no USB mass-storage device was found on any root port");
            return Err(());
        };

        Ok(XhciMassStorageDevice {
            controller: self,
            device: probed.device,
            bulk_in_ep: probed.config.bulk_in_ep,
            bulk_out_ep: probed.config.bulk_out_ep,
            bulk_in_index: probed.bulk_in_index,
            bulk_out_index: probed.bulk_out_index,
            max_packet_in: probed.config.max_packet_in,
            max_packet_out: probed.config.max_packet_out,
        })
    }

    /// Addresses the device sitting on an already-reset root port and, if it
    /// turns out to be a hub, walks that hub's downstream ports looking for a
    /// mass-storage device.
    fn probe_root_port_device(
        &mut self,
        port_id: u8,
        speed: UsbSpeed,
    ) -> Result<ProbedMassStorage, ()> {
        let slot_id = self.enable_slot()?;
        println!("xHCI: enabled slot {slot_id} for root port {port_id}");
        let mut device = self.allocate_device(slot_id, port_id, speed, 0, 0, 0, false)?;
        self.address_device(&mut device)?;
        println!("xHCI: addressed device on slot {slot_id}");

        let mut device_descriptor = [0u8; 18];
        self.get_descriptor(&mut device, USB_DT_DEVICE, 0, &mut device_descriptor)?;
        println!(
            "xHCI: device descriptor: class {:#04X} vendor {:04X}:{:04X}",
            device_descriptor[4],
            u16::from_le_bytes([device_descriptor[8], device_descriptor[9]]),
            u16::from_le_bytes([device_descriptor[10], device_descriptor[11]])
        );

        if device_descriptor[4] != USB_CLASS_HUB {
            return self.configure_mass_storage_device(device, speed);
        }

        println!(
            "xHCI: root port {} has a {} hub attached (protocol {})",
            port_id,
            Self::speed_name(speed),
            device_descriptor[6]
        );
        let hub = self.configure_hub_device(&mut device, &device_descriptor)?;
        self.probe_hub_downstream_ports(&mut device, &hub, port_id)
    }

    /// Tries every downstream port of `hub_device` in turn, so that a hub port
    /// holding a non-storage device (or a nested hub, which this driver does
    /// not traverse) does not hide a mass-storage device on a later port.
    fn probe_hub_downstream_ports(
        &mut self,
        hub_device: &mut XhciDevice,
        hub: &HubDescriptor,
        port_id: u8,
    ) -> Result<ProbedMassStorage, ()> {
        for downstream_port in 1..=hub.num_ports {
            let Some(downstream_speed) = self.reset_hub_downstream_port(hub_device, downstream_port)
            else {
                continue;
            };
            println!(
                "xHCI: hub downstream port {} has a {} device attached",
                downstream_port,
                Self::speed_name(downstream_speed)
            );
            match self.probe_hub_downstream_device(
                hub_device,
                hub,
                port_id,
                downstream_port,
                downstream_speed,
            ) {
                Ok(probed) => return Ok(probed),
                Err(()) => println!(
                    "xHCI: no usable mass-storage device on hub downstream port \
                     {downstream_port}, continuing the scan"
                ),
            }
        }
        println!("xHCI: no usable downstream device found behind the hub on root port {port_id}");
        Err(())
    }

    fn probe_hub_downstream_device(
        &mut self,
        hub_device: &mut XhciDevice,
        hub: &HubDescriptor,
        port_id: u8,
        downstream_port: u8,
        downstream_speed: UsbSpeed,
    ) -> Result<ProbedMassStorage, ()> {
        let child_slot_id = self.enable_slot()?;
        /* This driver only supports a single hub tier, so the route string is
         * just the hub port encoded in the low nibble. Deeper hub chains would
         * need to accumulate one 4-bit hop per level. */
        let route_string = min(downstream_port, 15) as u32;
        let (tt_hub_slot_id, tt_port_number, multi_tt) =
            if matches!(downstream_speed, UsbSpeed::Low | UsbSpeed::Full)
                && hub_device.speed == UsbSpeed::High
            {
                (hub_device.slot_id, downstream_port, hub.multi_tt)
            } else {
                (0, 0, false)
            };
        let mut downstream = self.allocate_device(
            child_slot_id,
            port_id,
            downstream_speed,
            route_string,
            tt_hub_slot_id,
            tt_port_number,
            multi_tt,
        )?;
        self.address_device(&mut downstream)?;
        let mut device_descriptor = [0u8; 18];
        self.get_descriptor(&mut downstream, USB_DT_DEVICE, 0, &mut device_descriptor)?;
        println!(
            "xHCI: downstream device descriptor: class {:#04X} vendor {:04X}:{:04X}",
            device_descriptor[4],
            u16::from_le_bytes([device_descriptor[8], device_descriptor[9]]),
            u16::from_le_bytes([device_descriptor[10], device_descriptor[11]])
        );
        if device_descriptor[4] == USB_CLASS_HUB {
            println!("xHCI: only one level of hub traversal is currently supported");
            return Err(());
        }
        self.configure_mass_storage_device(downstream, downstream_speed)
    }

    /// Reads `device`'s configuration descriptor and, when it exposes a
    /// Bulk-Only mass-storage interface, selects that configuration and sets
    /// up its bulk endpoints.
    fn configure_mass_storage_device(
        &mut self,
        mut device: XhciDevice,
        speed: UsbSpeed,
    ) -> Result<ProbedMassStorage, ()> {
        let mut config_header = [0u8; 9];
        self.get_descriptor(&mut device, USB_DT_CONFIG, 0, &mut config_header)?;
        let total_length = u16::from_le_bytes([config_header[2], config_header[3]]) as usize;
        if total_length > 512 || total_length < 9 {
            println!("xHCI: unsupported configuration descriptor length {total_length}");
            return Err(());
        }
        let mut configuration = [0u8; 512];
        self.get_descriptor(
            &mut device,
            USB_DT_CONFIG,
            0,
            &mut configuration[..total_length],
        )?;
        let configuration_value = configuration[5];

        let Some(config) =
            Self::parse_mass_storage_configuration(&configuration[..total_length], speed)
        else {
            println!("xHCI: no USB mass-storage interface found");
            return Err(());
        };

        self.set_configuration(&mut device, configuration_value)?;
        let (bulk_out_index, bulk_in_index) = self.configure_mass_storage_endpoints(
            &mut device,
            config.bulk_out_ep,
            config.bulk_in_ep,
            config.max_packet_out,
            config.max_packet_in,
            config.max_burst_out,
            config.max_burst_in,
            config.max_ep_index,
        )?;

        Ok(ProbedMassStorage {
            device,
            config,
            bulk_out_index,
            bulk_in_index,
        })
    }

    fn reset_controller(&mut self) -> Result<(), ()> {
        let status = self.read_operational32(0x04);
        if (status & USBSTS_HALT) == 0 {
            let cmd = self.read_operational32(0x00) & !USBCMD_RUN;
            self.write_operational32(0x00, cmd);
        }
        self.wait_operational_bits(0x04, USBSTS_HALT, USBSTS_HALT, HALT_TIMEOUT_US)?;

        let cmd = self.read_operational32(0x00) | USBCMD_RESET;
        self.write_operational32(0x00, cmd);
        self.wait_operational_bits(0x00, USBCMD_RESET, 0, RESET_TIMEOUT_US)?;
        self.wait_operational_bits(0x04, USBSTS_CNR, 0, RESET_TIMEOUT_US)
    }

    fn start_controller(&mut self) -> Result<(), ()> {
        self.dcbaa.invalidate();
        if let Some(sp) = self.scratchpad_array.as_ref() {
            sp.invalidate();
        }
        self.erst.invalidate();
        let dcbaa0 = unsafe { read_volatile(self.dcbaa.address as *const u64) };
        let sp0 = self
            .scratchpad_array
            .as_ref()
            .map(|r| unsafe { read_volatile(r.address as *const u64) })
            .unwrap_or(0);
        let erst_seg = unsafe { read_volatile(self.erst.address as *const u64) };
        println!(
            "xHCI: regs DCBAAP={:#X} CRCR={:#X} ERSTBA={:#X} ERDP={:#X} ERSTSZ={:#X}",
            Self::read64(self.operational_base + 0x30),
            Self::read64(self.operational_base + 0x18),
            Self::read64(self.runtime_base + 0x30),
            Self::read64(self.runtime_base + 0x38),
            self.read_runtime32(0x28),
        );
        println!(
            "xHCI: ram DCBAA[0]={:#X} SP[0]={:#X} ERST.seg={:#X}",
            dcbaa0, sp0, erst_seg
        );
        println!(
            "xHCI: pre-start USBSTS={:#X} USBCMD={:#X}",
            self.read_operational32(0x04),
            self.read_operational32(0x00)
        );
        let cmd = self.read_operational32(0x00) | USBCMD_RUN;
        self.write_operational32(0x00, cmd);
        for _ in 0..10 {
            Self::delay_ms(1);
            let sts = self.read_operational32(0x04);
            if (sts & USBSTS_HSE) != 0 {
                println!("xHCI: HSE set {:#X} shortly after R/S=1", sts);
                break;
            }
            if (sts & USBSTS_HALT) == 0 {
                break;
            }
        }
        self.wait_operational_bits(0x04, USBSTS_HALT, 0, HALT_TIMEOUT_US)?;
        self.write_runtime32(0x20, 0);
        self.write_runtime32(0x1C, 0);
        Ok(())
    }

    fn program_max_slots(&mut self) {
        let max_slots = self.hcsparams1 & HCS_MAX_SLOTS_MASK;
        let mut config = self.read_operational32(0x38);
        config &= !CONFIG_MAX_SLOTS_MASK;
        config |= max_slots;
        self.write_operational32(0x38, config);
    }

    fn initialize_memory_structures(&mut self) -> Result<(), ()> {
        unsafe { write_bytes(self.dcbaa.address as *mut u8, 0, self.dcbaa.size) };
        self.dcbaa.clean();
        self.write_operational64(0x30, to_bus(self.dcbaa.address));

        self.write_operational64(
            0x18,
            (to_bus(self.command_ring.region.address) & !CMD_RING_RSVD_BITS)
                | self.command_ring.cycle_state as u64,
        );

        unsafe { write_bytes(self.erst.address as *mut u8, 0, self.erst.size) };
        let erst_entry = unsafe { &mut *(self.erst.address as *mut ErstEntry) };
        erst_entry.seg_addr = to_bus(self.event_ring.region.address);
        erst_entry.seg_size = TRBS_PER_SEGMENT as u32;
        erst_entry.reserved = 0;
        self.erst.clean();

        self.write_runtime64(
            0x38,
            to_bus(self.event_ring.region.address) & !ERST_PTR_MASK,
        );
        self.write_runtime32(0x28, 1);
        self.write_runtime64(0x30, to_bus(self.erst.address) & !ERST_PTR_MASK);

        self.allocate_scratchpads()?;
        self.write_operational32(0x14, 0);
        println!(
            "xHCI: hcs1={:#X} hcs2={:#X} hcc={:#X} pagesz={:#X} nscratch={}",
            self.hcsparams1,
            self.hcsparams2,
            self.hccparams,
            self.page_size,
            (((self.hcsparams2 >> HCS_MAX_SCRATCHPAD_HI_SHIFT) & 0x3E0)
                | ((self.hcsparams2 >> HCS_MAX_SCRATCHPAD_LO_SHIFT) & 0x1F))
        );
        println!(
            "xHCI: dcbaa={:#X} cmd={:#X} evt={:#X} erst={:#X} sp_arr={:#X} sp_buf={:#X}",
            self.dcbaa.address,
            self.command_ring.region.address,
            self.event_ring.region.address,
            self.erst.address,
            self.scratchpad_array
                .as_ref()
                .map(|r| r.address)
                .unwrap_or(0),
            self.scratchpad_buffers
                .as_ref()
                .map(|r| r.address)
                .unwrap_or(0),
        );
        Ok(())
    }

    fn allocate_scratchpads(&mut self) -> Result<(), ()> {
        let num_scratchpads = (((self.hcsparams2 >> HCS_MAX_SCRATCHPAD_HI_SHIFT) & 0x3E0)
            | ((self.hcsparams2 >> HCS_MAX_SCRATCHPAD_LO_SHIFT) & 0x1F))
            as usize;
        if num_scratchpads == 0 {
            return Ok(());
        }

        let scratchpad_array = DmaRegion::new(num_scratchpads * size_of::<u64>(), 12)?;
        let scratchpad_buffers = DmaRegion::new(
            num_scratchpads * self.page_size,
            self.page_size.ilog2() as usize,
        )?;
        unsafe {
            write_bytes(
                scratchpad_array.address as *mut u8,
                0,
                scratchpad_array.size,
            )
        };
        unsafe {
            write_bytes(
                scratchpad_buffers.address as *mut u8,
                0,
                scratchpad_buffers.size,
            )
        };
        for i in 0..num_scratchpads {
            unsafe {
                write_volatile(
                    (scratchpad_array.address as *mut u64).add(i),
                    to_bus(scratchpad_buffers.address + i * self.page_size),
                )
            };
        }
        scratchpad_buffers.clean();
        scratchpad_array.clean();
        unsafe {
            write_volatile(
                self.dcbaa.address as *mut u64,
                to_bus(scratchpad_array.address),
            )
        };
        self.dcbaa.clean();
        self.scratchpad_array = Some(scratchpad_array);
        self.scratchpad_buffers = Some(scratchpad_buffers);
        Ok(())
    }

    /// Powers (if needed) and resets a single root port, returning the
    /// negotiated speed when a device is present and successfully enabled.
    /// Returns `None` -- rather than an error -- for an empty or unusable
    /// port, so that the caller can keep scanning the remaining root ports.
    fn reset_root_port(&mut self, port: u8, power_control: bool) -> Option<UsbSpeed> {
        let mut status = self.read_portsc(port);
        if power_control && (status & PORT_POWER) == 0 {
            let neutral = Self::port_state_to_neutral(status) | PORT_POWER;
            self.write_portsc(port, neutral);
            /*
             * A port that was just powered on needs time before the
             * hardware can report a device as connected (xHCI spec 4.19.1
             * / bPwrOn2PwrGood-style settle time, typically up to tens of
             * ms). Re-reading PORTSC immediately after the power-on write
             * risks seeing PORT_CONNECT still clear even though a device
             * is physically attached, so wait a bit before the very
             * first read used to decide whether to skip this port.
             */
            Self::delay_ms(20);
            status = self.read_portsc(port);
        }
        println!("xHCI: root port {} PORTSC {:#X}", port, status);
        if (status & PORT_CONNECT) == 0 {
            return None;
        }
        self.clear_port_change_bits(port, status);
        let neutral = Self::port_state_to_neutral(status) | PORT_RESET;
        self.write_portsc(port, neutral);
        if self
            .wait_until(
                || {
                    let value = self.read_portsc(port);
                    (value & PORT_RESET) == 0 && (value & PORT_CONNECT) != 0
                },
                POLL_TIMEOUT_US,
            )
            .is_err()
        {
            println!(
                "xHCI: root port {} reset timed out (PORTSC {:#X})",
                port,
                self.read_portsc(port)
            );
            return None;
        }
        status = self.read_portsc(port);
        self.clear_port_change_bits(port, status);
        if (status & PORT_PE) == 0 {
            println!("xHCI: port {port} did not enable after reset ({status:#X})");
            return None;
        }
        let speed = match status & DEV_SPEED_MASK {
            XDEV_LS => UsbSpeed::Low,
            XDEV_FS => UsbSpeed::Full,
            XDEV_HS => UsbSpeed::High,
            XDEV_SS => UsbSpeed::Super,
            _ => return None,
        };
        println!(
            "xHCI: root port {} enabled, speed {}",
            port,
            Self::speed_name(speed)
        );
        Some(speed)
    }

    fn enable_slot(&mut self) -> Result<u8, ()> {
        self.queue_command(0, 0, 0, TRB_ENABLE_SLOT)?;
        let event = self.wait_for_event(TRB_COMPLETION_EVENT, POLL_TIMEOUT_US)?;
        let completion = Self::completion_code(event[2]);
        let slot_id = ((event[3] >> 24) & 0xFF) as u8;
        self.acknowledge_event();
        if completion != COMP_SUCCESS || slot_id == 0 {
            println!("xHCI: Enable Slot failed with completion {completion}");
            return Err(());
        }
        Ok(slot_id)
    }

    fn allocate_device(
        &self,
        slot_id: u8,
        root_hub_port: u8,
        speed: UsbSpeed,
        route_string: u32,
        tt_hub_slot_id: u8,
        tt_port_number: u8,
        multi_tt: bool,
    ) -> Result<XhciDevice, ()> {
        let out_ctx = DmaRegion::new(self.device_context_size(), 12)?;
        let in_ctx = DmaRegion::new(self.input_context_size(), 12)?;
        let ep0_ring = TrbRing::new(true)?;
        unsafe { write_bytes(out_ctx.address as *mut u8, 0, out_ctx.size) };
        unsafe { write_bytes(in_ctx.address as *mut u8, 0, in_ctx.size) };
        out_ctx.clean();
        in_ctx.clean();
        unsafe {
            write_volatile(
                (self.dcbaa.address as *mut u64).add(slot_id as usize),
                to_bus(out_ctx.address),
            )
        };
        self.dcbaa.clean();
        Ok(XhciDevice {
            slot_id,
            root_hub_port,
            speed,
            route_string,
            tt_hub_slot_id,
            tt_port_number,
            multi_tt,
            /* USB2 devices may need EP0's size learned from the descriptor, but
             * xHCI already knows the correct default control maxpacket once port
             * speed is known. For SuperSpeed that value is always 512 bytes, so
             * this driver can issue its first full device-descriptor read
             * directly without a separate "read 8 bytes then Evaluate Context"
             * dance. */
            ep0_max_packet: match speed {
                UsbSpeed::Low => 8,
                UsbSpeed::Full | UsbSpeed::High => 64,
                UsbSpeed::Super => 512,
            },
            out_ctx,
            in_ctx,
            ep0_ring,
            bulk_out_ring: None,
            bulk_in_ring: None,
        })
    }

    fn address_device(&mut self, device: &mut XhciDevice) -> Result<(), ()> {
        let slot_ctx = self.slot_context_mut(&device.in_ctx, true);
        let ep0_ctx = self.endpoint_context_mut(&device.in_ctx, 0, true);
        let ctrl_ctx = self.input_control_context_mut(&device.in_ctx);

        unsafe { write_bytes(device.in_ctx.address as *mut u8, 0, device.in_ctx.size) };
        ctrl_ctx.drop_flags = 0;
        ctrl_ctx.add_flags = SLOT_FLAG | EP0_FLAG;
        slot_ctx.dev_info =
            device.route_string | Self::slot_speed_bits(device.speed) | Self::last_ctx(1);
        if device.multi_tt {
            slot_ctx.dev_info |= DEV_MTT;
        }
        slot_ctx.dev_info2 = (device.root_hub_port as u32) << ROOT_HUB_PORT_SHIFT;
        slot_ctx.tt_info = if device.tt_hub_slot_id != 0 {
            Self::tt_slot(device.tt_hub_slot_id as u32)
                | Self::tt_port(device.tt_port_number as u32)
        } else {
            0
        };
        slot_ctx.dev_state = 0;
        ep0_ctx.ep_info = 0;
        ep0_ctx.ep_info2 = Self::ep_type(CTRL_EP)
            | Self::max_packet(device.ep0_max_packet)
            | Self::max_burst(0)
            | Self::error_count(3);
        ep0_ctx.deq = to_bus(device.ep0_ring.region.address) | device.ep0_ring.cycle_state as u64;
        ep0_ctx.tx_info = 8 & EP_AVG_TRB_LENGTH_MASK;
        device.in_ctx.clean();

        self.queue_command(
            to_bus(device.in_ctx.address),
            device.slot_id,
            0,
            TRB_ADDR_DEV,
        )?;
        let event = self.wait_for_event(TRB_COMPLETION_EVENT, POLL_TIMEOUT_US)?;
        let completion = Self::completion_code(event[2]);
        let slot_id = ((event[3] >> 24) & 0xFF) as u8;
        self.acknowledge_event();
        if completion != COMP_SUCCESS || slot_id != device.slot_id {
            println!("xHCI: Address Device failed with completion {completion}");
            return Err(());
        }
        device.out_ctx.invalidate();
        Ok(())
    }

    fn get_descriptor(
        &mut self,
        device: &mut XhciDevice,
        descriptor_type: u8,
        descriptor_index: u8,
        buffer: &mut [u8],
    ) -> Result<usize, ()> {
        self.control_transfer(
            device,
            SetupPacket {
                request_type: USB_DIR_IN,
                request: USB_REQ_GET_DESCRIPTOR,
                value: ((descriptor_type as u16) << 8) | descriptor_index as u16,
                index: 0,
                length: buffer.len() as u16,
            },
            buffer.as_mut_ptr() as usize,
            buffer.len(),
        )
    }

    fn set_configuration(&mut self, device: &mut XhciDevice, configuration: u8) -> Result<(), ()> {
        let _ = self.control_transfer(
            device,
            SetupPacket {
                request_type: 0,
                request: USB_REQ_SET_CONFIGURATION,
                value: configuration as u16,
                index: 0,
                length: 0,
            },
            0,
            0,
        )?;
        Ok(())
    }

    fn set_interface(
        &mut self,
        device: &mut XhciDevice,
        interface: u8,
        alternate: u8,
    ) -> Result<(), ()> {
        let _ = self.control_transfer(
            device,
            SetupPacket {
                request_type: USB_RECIP_INTERFACE,
                request: USB_REQ_SET_INTERFACE,
                value: alternate as u16,
                index: interface as u16,
                length: 0,
            },
            0,
            0,
        )?;
        Ok(())
    }

    fn configure_hub_device(
        &mut self,
        hub_device: &mut XhciDevice,
        device_descriptor: &[u8; 18],
    ) -> Result<HubDescriptor, ()> {
        let mut config_header = [0u8; 9];
        self.get_descriptor(hub_device, USB_DT_CONFIG, 0, &mut config_header)?;
        let configuration_value = config_header[5];
        self.set_configuration(hub_device, configuration_value)?;
        Self::delay_ms(10);
        println!("xHCI: hub set to configuration {}", configuration_value);

        let mut multi_tt = false;
        if hub_device.speed == UsbSpeed::High && device_descriptor[6] == USB_HUB_PR_HS_MULTI_TT {
            match self.set_interface(hub_device, 0, 1) {
                Ok(()) => {
                    multi_tt = true;
                    println!("xHCI: hub enabled Multi-TT mode");
                }
                Err(()) => {
                    println!("xHCI: hub Multi-TT selection failed, using Single-TT mode");
                }
            }
        }

        let mut hub_descriptor = self.read_hub_descriptor(hub_device, multi_tt)?;
        self.update_hub_device(hub_device, &hub_descriptor)?;
        self.power_hub_ports(hub_device, &hub_descriptor)?;
        hub_descriptor.multi_tt = multi_tt;
        Ok(hub_descriptor)
    }

    fn read_hub_descriptor(
        &mut self,
        hub_device: &mut XhciDevice,
        multi_tt: bool,
    ) -> Result<HubDescriptor, ()> {
        let descriptor_type = if hub_device.speed == UsbSpeed::Super {
            USB_DT_SS_HUB
        } else {
            USB_DT_HUB
        };
        let mut header = [0u8; 7];
        let header_len = self.control_transfer(
            hub_device,
            SetupPacket {
                request_type: USB_DIR_IN | USB_RT_HUB,
                request: USB_REQ_GET_DESCRIPTOR,
                value: (descriptor_type as u16) << 8,
                index: 0,
                length: header.len() as u16,
            },
            header.as_mut_ptr() as usize,
            header.len(),
        )?;
        if header_len < header.len() {
            println!("xHCI: hub descriptor header was truncated ({header_len} bytes)");
            return Err(());
        }
        let descriptor_len = header[0] as usize;
        if descriptor_len < header.len() || descriptor_len > 32 {
            println!("xHCI: unsupported hub descriptor length {descriptor_len}");
            return Err(());
        }

        let mut buffer = [0u8; 32];
        let full_len = self.control_transfer(
            hub_device,
            SetupPacket {
                request_type: USB_DIR_IN | USB_RT_HUB,
                request: USB_REQ_GET_DESCRIPTOR,
                value: (descriptor_type as u16) << 8,
                index: 0,
                length: descriptor_len as u16,
            },
            buffer.as_mut_ptr() as usize,
            descriptor_len,
        )?;
        if full_len < descriptor_len {
            println!(
                "xHCI: hub descriptor read was short (got {full_len} expected {descriptor_len})"
            );
            return Err(());
        }

        let characteristics = u16::from_le_bytes([buffer[3], buffer[4]]);
        let hub = HubDescriptor {
            num_ports: buffer[2],
            power_on_to_good_ms: buffer[5] as u16 * 2,
            tt_think_time: ((characteristics & HUB_CHAR_TTTT) >> HUB_CHAR_TTTT_SHIFT) as u8,
            multi_tt,
        };
        println!(
            "xHCI: hub descriptor type {:#04X}, {} downstream ports, {} power switching, TT think time {}",
            descriptor_type,
            hub.num_ports,
            if (characteristics & HUB_CHAR_LPSM) == HUB_CHAR_INDV_PORT_LPSM {
                "per-port"
            } else {
                "ganged"
            },
            (hub.tt_think_time + 1) * 8
        );
        Ok(hub)
    }

    fn update_hub_device(
        &mut self,
        hub_device: &mut XhciDevice,
        hub_descriptor: &HubDescriptor,
    ) -> Result<(), ()> {
        unsafe {
            write_bytes(
                hub_device.in_ctx.address as *mut u8,
                0,
                hub_device.in_ctx.size,
            )
        };
        let out_slot = self.slot_context_mut(&hub_device.out_ctx, false);
        let in_slot = self.slot_context_mut(&hub_device.in_ctx, true);
        unsafe {
            copy_nonoverlapping(
                out_slot as *const SlotContext,
                in_slot as *mut SlotContext,
                1,
            );
        }

        let ctrl_ctx = self.input_control_context_mut(&hub_device.in_ctx);
        ctrl_ctx.drop_flags = 0;
        ctrl_ctx.add_flags = SLOT_FLAG;
        in_slot.dev_info |= DEV_HUB;
        if hub_descriptor.multi_tt {
            in_slot.dev_info |= DEV_MTT;
        } else {
            in_slot.dev_info &= !DEV_MTT;
        }
        in_slot.dev_info2 |= Self::max_ports(hub_descriptor.num_ports as u32);
        if hub_device.speed == UsbSpeed::High {
            in_slot.tt_info |= Self::tt_think_time(hub_descriptor.tt_think_time as u32);
        }
        in_slot.dev_state = 0;
        hub_device.in_ctx.clean();

        self.queue_command(
            to_bus(hub_device.in_ctx.address),
            hub_device.slot_id,
            0,
            TRB_CONFIG_EP,
        )?;
        let event = self.wait_for_event(TRB_COMPLETION_EVENT, POLL_TIMEOUT_US)?;
        let completion = Self::completion_code(event[2]);
        let slot_id = ((event[3] >> 24) & 0xFF) as u8;
        self.acknowledge_event();
        if completion != COMP_SUCCESS || slot_id != hub_device.slot_id {
            println!("xHCI: hub Configure Endpoint failed with completion {completion}");
            return Err(());
        }
        hub_device.out_ctx.invalidate();
        println!(
            "xHCI: hub slot {} updated for {} downstream ports",
            hub_device.slot_id, hub_descriptor.num_ports
        );
        Ok(())
    }

    fn power_hub_ports(
        &mut self,
        hub_device: &mut XhciDevice,
        hub_descriptor: &HubDescriptor,
    ) -> Result<(), ()> {
        println!(
            "xHCI: powering {} hub downstream ports",
            hub_descriptor.num_ports
        );
        for port in 1..=hub_descriptor.num_ports {
            if hub_device.speed == UsbSpeed::Super {
                self.hub_set_port_feature(hub_device, port, USB_PORT_FEAT_RESET)?;
            }
            self.hub_set_port_feature(hub_device, port, USB_PORT_FEAT_POWER)?;
        }
        Self::delay_ms(hub_descriptor.power_on_to_good_ms.max(100));
        Ok(())
    }

    /// Resets a single hub downstream port, returning the negotiated speed
    /// when a device is present and successfully enabled. Returns `None` for
    /// an empty or unusable port so the caller can try the next one.
    fn reset_hub_downstream_port(
        &mut self,
        hub_device: &mut XhciDevice,
        port: u8,
    ) -> Option<UsbSpeed> {
        let status = self.hub_get_port_status(hub_device, port).ok()?;
        println!(
            "xHCI: hub port {} status {:#06X} change {:#06X}",
            port, status.status, status.change
        );
        if (status.status & USB_PORT_STAT_CONNECTION) == 0 {
            return None;
        }
        println!("xHCI: resetting hub downstream port {}", port);
        let enumerated = self.reset_hub_port(hub_device, port).ok()?;
        if (enumerated.status & USB_PORT_STAT_CONNECTION) == 0
            || (enumerated.status & USB_PORT_STAT_ENABLE) == 0
        {
            println!(
                "xHCI: hub port {} failed to enable after reset ({:#06X})",
                port, enumerated.status
            );
            return None;
        }
        Self::hub_port_speed(hub_device.speed, enumerated.status).ok()
    }

    fn reset_hub_port(
        &mut self,
        hub_device: &mut XhciDevice,
        port: u8,
    ) -> Result<HubPortStatus, ()> {
        let mut delay_us = HUB_SHORT_RESET_DELAY_US;
        for attempt in 1..=5 {
            self.hub_set_port_feature(hub_device, port, USB_PORT_FEAT_RESET)?;
            Self::delay_us(delay_us);
            let status = self.wait_for_hub_port_reset(hub_device, port)?;
            println!(
                "xHCI: hub port {} reset attempt {} -> status {:#06X} change {:#06X}",
                port, attempt, status.status, status.change
            );
            if (status.status & USB_PORT_STAT_ENABLE) != 0 {
                self.hub_clear_port_feature(hub_device, port, USB_PORT_FEAT_C_RESET)?;
                return Ok(status);
            }
            delay_us = HUB_LONG_RESET_DELAY_US;
        }
        Err(())
    }

    fn wait_for_hub_port_reset(
        &mut self,
        hub_device: &mut XhciDevice,
        port: u8,
    ) -> Result<HubPortStatus, ()> {
        let deadline = Self::deadline(POLL_TIMEOUT_US);
        loop {
            let status = self.hub_get_port_status(hub_device, port)?;
            if (status.status & USB_PORT_STAT_CONNECTION) == 0 {
                return Ok(status);
            }
            if (status.status & USB_PORT_STAT_RESET) == 0
                && ((status.change & USB_PORT_STAT_C_RESET) != 0
                    || (status.status & USB_PORT_STAT_ENABLE) != 0)
            {
                return Ok(status);
            }
            if Self::deadline_passed(deadline) {
                return Err(());
            }
            core::hint::spin_loop();
        }
    }

    fn hub_get_port_status(
        &mut self,
        hub_device: &mut XhciDevice,
        port: u8,
    ) -> Result<HubPortStatus, ()> {
        let mut buffer = [0u8; 4];
        let length = self.control_transfer(
            hub_device,
            SetupPacket {
                request_type: USB_DIR_IN | USB_RT_PORT,
                request: USB_REQ_GET_STATUS,
                value: 0,
                index: port as u16,
                length: buffer.len() as u16,
            },
            buffer.as_mut_ptr() as usize,
            buffer.len(),
        )?;
        if length < buffer.len() {
            println!("xHCI: short hub port status read on port {}", port);
            return Err(());
        }
        Ok(HubPortStatus {
            status: u16::from_le_bytes([buffer[0], buffer[1]]),
            change: u16::from_le_bytes([buffer[2], buffer[3]]),
        })
    }

    fn hub_set_port_feature(
        &mut self,
        hub_device: &mut XhciDevice,
        port: u8,
        feature: u16,
    ) -> Result<(), ()> {
        let _ = self.control_transfer(
            hub_device,
            SetupPacket {
                request_type: USB_RT_PORT,
                request: USB_REQ_SET_FEATURE,
                value: feature,
                index: port as u16,
                length: 0,
            },
            0,
            0,
        )?;
        Ok(())
    }

    fn hub_clear_port_feature(
        &mut self,
        hub_device: &mut XhciDevice,
        port: u8,
        feature: u16,
    ) -> Result<(), ()> {
        let _ = self.control_transfer(
            hub_device,
            SetupPacket {
                request_type: USB_RT_PORT,
                request: USB_REQ_CLEAR_FEATURE,
                value: feature,
                index: port as u16,
                length: 0,
            },
            0,
            0,
        )?;
        Ok(())
    }

    fn configure_mass_storage_endpoints(
        &mut self,
        device: &mut XhciDevice,
        bulk_out_ep: u8,
        bulk_in_ep: u8,
        max_packet_out: u16,
        max_packet_in: u16,
        max_burst_out: u8,
        max_burst_in: u8,
        max_ep_index: usize,
    ) -> Result<(usize, usize), ()> {
        let bulk_out_index = Self::endpoint_index(bulk_out_ep);
        let bulk_in_index = Self::endpoint_index(bulk_in_ep);
        let bulk_out_ring = TrbRing::new(true)?;
        let bulk_in_ring = TrbRing::new(true)?;
        device.bulk_out_ring = Some(bulk_out_ring);
        device.bulk_in_ring = Some(bulk_in_ring);

        unsafe { write_bytes(device.in_ctx.address as *mut u8, 0, device.in_ctx.size) };
        let out_slot = self.slot_context_mut(&device.out_ctx, false);
        let out_ep0 = self.endpoint_context_mut(&device.out_ctx, 0, false);
        let in_slot = self.slot_context_mut(&device.in_ctx, true);
        let in_ep0 = self.endpoint_context_mut(&device.in_ctx, 0, true);
        unsafe {
            copy_nonoverlapping(
                out_slot as *const SlotContext,
                in_slot as *mut SlotContext,
                1,
            );
            copy_nonoverlapping(
                out_ep0 as *const EndpointContext,
                in_ep0 as *mut EndpointContext,
                1,
            );
        }

        let ctrl_ctx = self.input_control_context_mut(&device.in_ctx);
        ctrl_ctx.drop_flags = 0;
        ctrl_ctx.add_flags = SLOT_FLAG | (1 << (bulk_out_index + 1)) | (1 << (bulk_in_index + 1));
        in_slot.dev_info &= !(0x1F << LAST_CTX_SHIFT);
        in_slot.dev_info |= Self::last_ctx((max_ep_index + 1) as u32);

        let out_ring = device.bulk_out_ring.as_ref().ok_or(())?;
        let in_ring = device.bulk_in_ring.as_ref().ok_or(())?;
        let out_ctx = self.endpoint_context_mut(&device.in_ctx, bulk_out_index, true);
        out_ctx.ep_info = 0;
        out_ctx.ep_info2 = Self::ep_type(BULK_OUT_EP)
            | Self::max_packet(max_packet_out)
            | Self::max_burst(max_burst_out as u32)
            | Self::error_count(3);
        out_ctx.deq = to_bus(out_ring.region.address) | out_ring.cycle_state as u64;
        out_ctx.tx_info = max_packet_out as u32;

        let in_ctx = self.endpoint_context_mut(&device.in_ctx, bulk_in_index, true);
        in_ctx.ep_info = 0;
        in_ctx.ep_info2 = Self::ep_type(BULK_IN_EP)
            | Self::max_packet(max_packet_in)
            | Self::max_burst(max_burst_in as u32)
            | Self::error_count(3);
        in_ctx.deq = to_bus(in_ring.region.address) | in_ring.cycle_state as u64;
        in_ctx.tx_info = max_packet_in as u32;
        device.in_ctx.clean();

        self.queue_command(
            to_bus(device.in_ctx.address),
            device.slot_id,
            0,
            TRB_CONFIG_EP,
        )?;
        let event = self.wait_for_event(TRB_COMPLETION_EVENT, POLL_TIMEOUT_US)?;
        let completion = Self::completion_code(event[2]);
        let slot_id = ((event[3] >> 24) & 0xFF) as u8;
        self.acknowledge_event();
        if completion != COMP_SUCCESS || slot_id != device.slot_id {
            println!("xHCI: Configure Endpoint failed with completion {completion}");
            return Err(());
        }
        device.out_ctx.invalidate();
        Ok((bulk_out_index, bulk_in_index))
    }

    fn control_transfer(
        &mut self,
        device: &mut XhciDevice,
        setup: SetupPacket,
        buffer_address: usize,
        length: usize,
    ) -> Result<usize, ()> {
        let ring = &mut device.ep0_ring;
        let start_index = ring.enqueue_index;
        let start_cycle = ring.cycle_state;
        let mut setup_flags = Self::trb_type(TRB_SETUP) | TRB_IDT;
        if self.hci_version >= 0x100 && length > 0 {
            setup_flags |= if (setup.request_type & USB_DIR_IN) != 0 {
                3 << 16
            } else {
                2 << 16
            };
        }
        if start_cycle == 0 {
            setup_flags |= TRB_CYCLE;
        }
        let setup_fields = [
            (setup.request_type as u32)
                | ((setup.request as u32) << 8)
                | ((setup.value as u32) << 16),
            (setup.index as u32) | ((setup.length as u32) << 16),
            8,
            setup_flags,
        ];
        Self::push_trb_raw(ring, setup_fields);

        let mut bounce = if length > 0 && Self::needs_bounce_buffer(buffer_address, length) {
            Some(BounceBuffer::new(length)?)
        } else {
            None
        };
        let dma_buffer_address = bounce
            .as_ref()
            .map(|b| b.address())
            .unwrap_or(buffer_address);

        if length > 0 {
            /* U-Boot's xhci_ctrl_tx flushes (cleans) the buffer before every
             * data-stage transfer regardless of direction, not only for OUT.
             * This is required even for IN transfers: without it, a stale
             * dirty cache line covering this buffer (e.g. from the caller's
             * zero-initialization) can be evicted after the controller's DMA
             * write lands in RAM, silently clobbering the received data with
             * the old cached contents before we get to invalidate it. */
            if (setup.request_type & USB_DIR_IN) == 0 {
                if let Some(bounce) = bounce.as_mut() {
                    bounce.copy_from(buffer_address, length);
                }
            }
            unsafe { asm::clean_dcache_range(dma_buffer_address, length) };
            let mut data_flags = Self::trb_type(TRB_DATA);
            if (setup.request_type & USB_DIR_IN) != 0 {
                data_flags |= TRB_DIR_IN | TRB_ISP;
            }
            data_flags |= ring.cycle_state;
            let remainder = Self::transfer_trb_remainder(
                length,
                length,
                length,
                device.ep0_max_packet as usize,
            );
            let data_bus = to_bus(dma_buffer_address);
            let data_fields = [
                data_bus as u32,
                (data_bus >> 32) as u32,
                Self::trb_len(length) | Self::trb_td_size(remainder),
                data_flags,
            ];
            Self::push_trb_raw(ring, data_fields);
        }

        let mut status_flags = Self::trb_type(TRB_STATUS) | TRB_IOC | ring.cycle_state;
        if length == 0 || (setup.request_type & USB_DIR_IN) == 0 {
            status_flags |= TRB_DIR_IN;
        }
        let status_fields = [0, 0, 0, status_flags];
        Self::push_trb_raw(ring, status_fields);

        Self::commit_first_trb(ring, start_index, start_cycle);
        self.ring_doorbell(device.slot_id, 0);
        let event = self.wait_for_transfer_event(device.slot_id, 0, POLL_TIMEOUT_US)?;
        let completion = Self::completion_code(event[2]);
        let residue = Self::event_residue(event[2]) as usize;
        self.acknowledge_event();
        if completion != COMP_SUCCESS && completion != COMP_SHORT_TX {
            println!("xHCI: control transfer failed with completion {completion}");
            return Err(());
        }
        if length > 0 && (setup.request_type & USB_DIR_IN) != 0 {
            unsafe { asm::invalidate_dcache_range(dma_buffer_address, length) };
            if let Some(bounce) = bounce.as_ref() {
                bounce.copy_to(buffer_address, length);
            }
        }
        Ok(length.saturating_sub(residue))
    }

    /// The ring holds `TRBS_PER_SEGMENT` slots, one of which is permanently
    /// occupied by the Link TRB that makes it circular, so at most this many
    /// data TRBs can be outstanding at once. A single `submit_bulk_transfer`
    /// call builds its whole Transfer Descriptor (TD) up front and only
    /// rings the doorbell once, after every TRB has been written -- the
    /// controller does not start consuming any of them until then. If a
    /// transfer needed more TRBs than fit in the ring, writing the later
    /// ones would silently wrap around and overwrite the earlier ones
    /// before the controller ever got a chance to read them, corrupting the
    /// TD (confirmed on real Raspberry Pi 4 hardware: a ~24 MiB single-TD
    /// USB3 bulk read -- far more than the ~63*64KiB the ring can hold --
    /// hung with "timed out waiting for a transfer event"). Splitting a
    /// large transfer into multiple back-to-back "waves" of at most this
    /// many chained TRBs each, doorbell-and-waited-for individually, avoids
    /// this without limiting how large a single logical transfer can be.
    const MAX_TRBS_PER_WAVE: usize = TRBS_PER_SEGMENT - 1;

    /// Computes how many bytes starting at `address` can be covered by at
    /// most [`Self::MAX_TRBS_PER_WAVE`] TRBs, each of which is additionally
    /// capped at 64KiB and may never straddle a 64KiB boundary (an xHCI TRB
    /// data-buffer-pointer restriction).
    fn max_wave_length(mut address: usize, remaining: usize) -> usize {
        let mut total = 0usize;
        for _ in 0..Self::MAX_TRBS_PER_WAVE {
            if total >= remaining {
                break;
            }
            let boundary_limit = 0x1_0000usize - (address & 0xFFFF);
            let chunk = min(
                remaining - total,
                if boundary_limit == 0 {
                    0x1_0000
                } else {
                    boundary_limit
                },
            );
            total += chunk;
            address += chunk;
        }
        total
    }

    /// Builds and executes a single TD (chain of TRBs, all queued before one
    /// doorbell ring) covering exactly `wave_length` bytes starting at
    /// `dma_address`, which must be `<= Self::max_wave_length(dma_address,
    /// wave_length)`. Returns the number of bytes actually transferred and
    /// the event's completion code.
    fn submit_bulk_wave(
        &mut self,
        slot_id: u8,
        ep_index: usize,
        ring: &mut TrbRing,
        dma_address: usize,
        wave_length: usize,
        max_packet_size: u16,
        is_in: bool,
    ) -> Result<(usize, u32), ()> {
        let start_index = ring.enqueue_index;
        let start_cycle = ring.cycle_state;
        let mut transferred = 0usize;
        let mut remaining = wave_length;
        while remaining > 0 {
            let current_address = dma_address + transferred;
            let boundary_limit = 0x1_0000usize - (current_address & 0xFFFF);
            let chunk = min(
                remaining,
                if boundary_limit == 0 {
                    0x1_0000
                } else {
                    boundary_limit
                },
            );
            let more = remaining > chunk;
            let mut flags = Self::trb_type(TRB_NORMAL);
            if transferred == 0 {
                if start_cycle == 0 {
                    flags |= TRB_CYCLE;
                }
            } else {
                flags |= ring.cycle_state;
            }
            if is_in {
                flags |= TRB_ISP;
            }
            if more {
                flags |= TRB_CHAIN;
            } else {
                flags |= TRB_IOC;
            }
            let remainder = Self::transfer_trb_remainder(
                transferred,
                chunk,
                wave_length,
                max_packet_size as usize,
            );
            let current_bus = to_bus(current_address);
            let fields = [
                current_bus as u32,
                (current_bus >> 32) as u32,
                Self::trb_len(chunk) | Self::trb_td_size(remainder),
                flags,
            ];
            Self::push_trb_raw(ring, fields);
            transferred += chunk;
            remaining -= chunk;
        }
        Self::commit_first_trb(ring, start_index, start_cycle);
        self.ring_doorbell(slot_id, ep_index);
        let event = self.wait_for_transfer_event(slot_id, ep_index, POLL_TIMEOUT_US)?;
        let completion = Self::completion_code(event[2]);
        let residue = Self::event_residue(event[2]) as usize;
        self.acknowledge_event();
        Ok((wave_length.saturating_sub(residue), completion))
    }

    fn submit_bulk_transfer(
        &mut self,
        slot_id: u8,
        ep_index: usize,
        ring: &mut TrbRing,
        buffer_address: usize,
        length: usize,
        max_packet_size: u16,
        is_in: bool,
    ) -> Result<usize, ()> {
        if length == 0 {
            return Ok(0);
        }
        let mut bounce = if Self::needs_bounce_buffer(buffer_address, length) {
            Some(BounceBuffer::new(length)?)
        } else {
            None
        };
        let dma_buffer_address = bounce
            .as_ref()
            .map(|b| b.address())
            .unwrap_or(buffer_address);
        /* Always clean (write back) the buffer before the transfer, even for
         * IN, matching U-Boot's xhci_bulk_tx. Otherwise a stale dirty cache
         * line over this buffer can be evicted after the controller's DMA
         * write lands in RAM, clobbering the received data before we
         * invalidate and read it. */
        if !is_in {
            if let Some(bounce) = bounce.as_mut() {
                bounce.copy_from(buffer_address, length);
            }
        }
        unsafe { asm::clean_dcache_range(dma_buffer_address, length) };

        let mut transferred_total = 0usize;
        let mut offset = 0usize;
        while offset < length {
            let wave_address = dma_buffer_address + offset;
            let wave_length = Self::max_wave_length(wave_address, length - offset);
            let (wave_transferred, completion) = self.submit_bulk_wave(
                slot_id,
                ep_index,
                ring,
                wave_address,
                wave_length,
                max_packet_size,
                is_in,
            )?;
            if completion != COMP_SUCCESS && completion != COMP_SHORT_TX {
                println!("xHCI: bulk transfer failed with completion {completion}");
                if is_in {
                    unsafe { asm::invalidate_dcache_range(dma_buffer_address, length) };
                    if let Some(bounce) = bounce.as_ref() {
                        bounce.copy_to(buffer_address, length);
                    }
                }
                return Err(());
            }
            transferred_total += wave_transferred;
            offset += wave_length;
            /* A short transfer (fewer bytes than requested) ends the overall
             * transfer early, same as a single-TD transfer would: there is
             * no more data to wait for from the device. */
            if wave_transferred < wave_length {
                break;
            }
        }
        if is_in {
            unsafe { asm::invalidate_dcache_range(dma_buffer_address, length) };
            if let Some(bounce) = bounce.as_ref() {
                bounce.copy_to(buffer_address, length);
            }
        }
        Ok(transferred_total)
    }

    fn wait_for_transfer_event(
        &mut self,
        slot_id: u8,
        ep_index: usize,
        timeout_us: u64,
    ) -> Result<[u32; 4], ()> {
        let deadline = Self::deadline(timeout_us);
        loop {
            if let Ok(event) = self.peek_event() {
                let event_type = Self::trb_field_to_type(event[3]);
                if event_type == TRB_TRANSFER_EVENT {
                    let event_slot = ((event[3] >> 24) & 0xFF) as u8;
                    let event_ep_id = (((event[3] >> 16) & 0x1F) as usize).saturating_sub(1);
                    if event_slot == slot_id && event_ep_id == ep_index {
                        return Ok(event);
                    }
                }
                self.acknowledge_event();
            }
            if Self::deadline_passed(deadline) {
                println!(
                    "xHCI: timed out waiting for a transfer event (slot {slot_id} ep {ep_index}, USBSTS={:#X})",
                    self.read_operational32(0x04)
                );
                return Err(());
            }
        }
    }

    fn queue_command(
        &mut self,
        address: u64,
        slot_id: u8,
        ep_index: usize,
        command: u32,
    ) -> Result<(), ()> {
        let ring = &mut self.command_ring;
        let fields = [
            address as u32,
            (address >> 32) as u32,
            0,
            Self::trb_type(command)
                | ((slot_id as u32) << 24)
                | ((((ep_index + 1) & 0x1F) as u32) << 16)
                | ring.cycle_state,
        ];
        Self::push_trb_raw(ring, fields);
        unsafe { asm::dsb_sy() };
        self.ring_doorbell(0, 0);
        Ok(())
    }

    fn wait_for_event(&mut self, expected_type: u32, timeout_us: u64) -> Result<[u32; 4], ()> {
        let deadline = Self::deadline(timeout_us);
        loop {
            if let Ok(event) = self.peek_event() {
                let event_type = Self::trb_field_to_type(event[3]);
                if event_type == expected_type {
                    return Ok(event);
                }
                self.acknowledge_event();
            }
            if Self::deadline_passed(deadline) {
                println!(
                    "xHCI: timed out waiting for event type {expected_type} (USBSTS={:#X}, USBCMD={:#X})",
                    self.read_operational32(0x04),
                    self.read_operational32(0x00)
                );
                return Err(());
            }
        }
    }

    fn peek_event(&mut self) -> Result<[u32; 4], ()> {
        self.event_ring
            .region
            .invalidate_range(self.event_ring.dequeue_index * 16, 16);
        let address = self.event_ring.region.address + self.event_ring.dequeue_index * 16;
        let event = [
            unsafe { read_volatile(address as *const u32) },
            unsafe { read_volatile((address + 4) as *const u32) },
            unsafe { read_volatile((address + 8) as *const u32) },
            unsafe { read_volatile((address + 12) as *const u32) },
        ];
        if (event[3] & TRB_CYCLE) != self.event_ring.cycle_state {
            return Err(());
        }
        Ok(event)
    }

    fn acknowledge_event(&mut self) {
        self.event_ring.dequeue_index += 1;
        if self.event_ring.dequeue_index == TRBS_PER_SEGMENT {
            self.event_ring.dequeue_index = 0;
            self.event_ring.cycle_state ^= 1;
        }
        let dequeue = self.event_ring.region.address + self.event_ring.dequeue_index * 16;
        self.write_runtime64(0x38, (to_bus(dequeue) & !ERST_PTR_MASK) | ERST_EHB);
    }

    fn push_trb_raw(ring: &mut TrbRing, fields: [u32; 4]) {
        let trb = (ring.region.address + ring.enqueue_index * 16) as *mut u32;
        unsafe {
            write_volatile(trb.add(0), fields[0]);
            write_volatile(trb.add(1), fields[1]);
            write_volatile(trb.add(2), fields[2]);
            write_volatile(trb.add(3), fields[3]);
        }
        ring.region.clean_range(ring.enqueue_index * 16, 16);
        ring.advance_enqueue();
    }

    fn commit_first_trb(ring: &TrbRing, start_index: usize, start_cycle: u32) {
        let field3_address = ring.region.address + start_index * 16 + 12;
        let current = unsafe { read_volatile(field3_address as *const u32) };
        let updated = if start_cycle != 0 {
            current | TRB_CYCLE
        } else {
            current & !TRB_CYCLE
        };
        unsafe { write_volatile(field3_address as *mut u32, updated) };
        ring.region.clean_range(start_index * 16, 16);
        unsafe { asm::dsb_sy() };
    }

    fn ring_doorbell(&self, slot_id: u8, ep_index: usize) {
        let value = if slot_id == 0 {
            0
        } else {
            ((ep_index + 1) & 0xFF) as u32
        };
        Self::write32(self.doorbell_base + slot_id as usize * 4, value);
    }

    fn input_control_context_mut<'a>(&self, ctx: &'a DmaRegion) -> &'a mut InputControlContext {
        unsafe { &mut *(ctx.address as *mut InputControlContext) }
    }

    fn slot_context_mut<'a>(&self, ctx: &'a DmaRegion, is_input: bool) -> &'a mut SlotContext {
        let offset = if is_input { self.context_size() } else { 0 };
        unsafe { &mut *((ctx.address + offset) as *mut SlotContext) }
    }

    fn endpoint_context_mut<'a>(
        &self,
        ctx: &'a DmaRegion,
        ep_index: usize,
        is_input: bool,
    ) -> &'a mut EndpointContext {
        let mut index = ep_index + 1;
        if is_input {
            index += 1;
        }
        unsafe { &mut *((ctx.address + index * self.context_size()) as *mut EndpointContext) }
    }

    fn context_size(&self) -> usize {
        if (self.hccparams & HCC_64BYTE_CONTEXT) != 0 {
            64
        } else {
            32
        }
    }

    fn device_context_size(&self) -> usize {
        (MAX_EP_CTX_NUM + 1) * self.context_size()
    }

    fn input_context_size(&self) -> usize {
        self.device_context_size() + self.context_size()
    }

    fn parse_mass_storage_configuration(
        descriptor: &[u8],
        speed: UsbSpeed,
    ) -> Option<MassStorageConfiguration> {
        let mut offset = 0usize;
        let mut active_mass_storage = false;
        let mut bulk_in = None;
        let mut bulk_out = None;
        let mut max_packet_in = 0u16;
        let mut max_packet_out = 0u16;
        let mut max_burst_in = 0u8;
        let mut max_burst_out = 0u8;
        let mut max_ep_index = 0usize;
        while offset + 2 <= descriptor.len() {
            let length = descriptor[offset] as usize;
            if length == 0 || offset + length > descriptor.len() {
                break;
            }
            match descriptor[offset + 1] {
                USB_DT_INTERFACE if length >= 9 => {
                    active_mass_storage = descriptor[offset + 5] == 0x08
                        && descriptor[offset + 6] == 0x06
                        && descriptor[offset + 7] == 0x50;
                }
                USB_DT_ENDPOINT if active_mass_storage && length >= 7 => {
                    let ep_address = descriptor[offset + 2];
                    let attributes = descriptor[offset + 3] & 0x3;
                    if attributes == USB_ENDPOINT_XFER_BULK {
                        let max_packet =
                            u16::from_le_bytes([descriptor[offset + 4], descriptor[offset + 5]]);
                        let max_burst = if speed == UsbSpeed::Super {
                            Self::parse_superspeed_endpoint_companion(
                                descriptor, offset, ep_address,
                            )
                        } else {
                            0
                        };
                        let ep_index = Self::endpoint_index(ep_address);
                        max_ep_index = max_ep_index.max(ep_index);
                        if (ep_address & USB_ENDPOINT_DIR_MASK) != 0 {
                            bulk_in = Some(ep_address);
                            max_packet_in = max_packet;
                            max_burst_in = max_burst;
                        } else {
                            bulk_out = Some(ep_address);
                            max_packet_out = max_packet;
                            max_burst_out = max_burst;
                        }
                    }
                }
                _ => {}
            }
            offset += length;
        }
        Some(MassStorageConfiguration {
            bulk_in_ep: bulk_in?,
            bulk_out_ep: bulk_out?,
            max_packet_in,
            max_packet_out,
            max_burst_in,
            max_burst_out,
            max_ep_index,
        })
    }

    fn parse_superspeed_endpoint_companion(
        descriptor: &[u8],
        endpoint_offset: usize,
        endpoint_address: u8,
    ) -> u8 {
        let companion_offset = endpoint_offset + descriptor[endpoint_offset] as usize;
        if companion_offset + 2 > descriptor.len() {
            println!(
                "xHCI: SuperSpeed bulk EP {endpoint_address:#04X} missing companion descriptor"
            );
            return 0;
        }

        let companion_length = descriptor[companion_offset] as usize;
        let companion_type = descriptor[companion_offset + 1];
        if companion_type != USB_DT_SS_EP_COMPANION {
            println!(
                "xHCI: SuperSpeed bulk EP {endpoint_address:#04X} missing SS companion descriptor (type {companion_type:#04X})"
            );
            return 0;
        }
        if companion_length < 6 || companion_offset + companion_length > descriptor.len() {
            println!(
                "xHCI: SuperSpeed bulk EP {endpoint_address:#04X} has malformed SS companion length {companion_length}"
            );
            return 0;
        }

        descriptor[companion_offset + 2]
    }

    fn endpoint_index(endpoint_address: u8) -> usize {
        let number = (endpoint_address & 0x0F) as usize;
        if (endpoint_address & USB_ENDPOINT_DIR_MASK) != 0 {
            number * 2
        } else {
            (number * 2).saturating_sub(1)
        }
    }

    fn read_portsc(&self, port_id: u8) -> u32 {
        self.read_operational32(0x400 + (port_id as usize - 1) * 0x10)
    }

    fn write_portsc(&self, port_id: u8, value: u32) {
        self.write_operational32(0x400 + (port_id as usize - 1) * 0x10, value);
    }

    fn clear_port_change_bits(&self, port_id: u8, status: u32) {
        self.write_portsc(
            port_id,
            Self::port_state_to_neutral(status)
                | PORT_CSC
                | PORT_PEC
                | PORT_WRC
                | PORT_OCC
                | PORT_RC
                | PORT_PLC
                | PORT_CEC,
        );
    }

    fn port_state_to_neutral(status: u32) -> u32 {
        status & (XHCI_PORT_RO | XHCI_PORT_RWS)
    }

    fn wait_until<F: Fn() -> bool>(&self, condition: F, timeout_us: u64) -> Result<(), ()> {
        let deadline = Self::deadline(timeout_us);
        while !condition() {
            if Self::deadline_passed(deadline) {
                return Err(());
            }
            core::hint::spin_loop();
        }
        Ok(())
    }

    fn wait_operational_bits(
        &self,
        offset: usize,
        mask: u32,
        expected: u32,
        timeout_us: u64,
    ) -> Result<(), ()> {
        self.wait_until(
            || (self.read_operational32(offset) & mask) == expected,
            timeout_us,
        )
    }

    fn read_operational32(&self, offset: usize) -> u32 {
        Self::read32(self.operational_base + offset)
    }

    fn write_operational32(&self, offset: usize, value: u32) {
        Self::write32(self.operational_base + offset, value)
    }

    fn write_operational64(&self, offset: usize, value: u64) {
        Self::write64(self.operational_base + offset, value)
    }

    fn write_runtime32(&self, offset: usize, value: u32) {
        Self::write32(self.runtime_base + offset, value)
    }

    fn write_runtime64(&self, offset: usize, value: u64) {
        Self::write64(self.runtime_base + offset, value)
    }

    fn read32(address: usize) -> u32 {
        unsafe { read_volatile(address as *const u32) }
    }

    fn read64(address: usize) -> u64 {
        let low = Self::read32(address) as u64;
        let high = Self::read32(address + 4) as u64;
        (high << 32) | low
    }

    fn read_runtime32(&self, offset: usize) -> u32 {
        Self::read32(self.runtime_base + offset)
    }

    fn write32(address: usize, value: u32) {
        unsafe { write_volatile(address as *mut u32, value) };
    }

    fn write64(address: usize, value: u64) {
        unsafe {
            write_volatile(address as *mut u32, value as u32);
            write_volatile((address + 4) as *mut u32, (value >> 32) as u32);
        }
    }

    fn last_ctx(endpoint_count: u32) -> u32 {
        endpoint_count << LAST_CTX_SHIFT
    }

    fn slot_speed_bits(speed: UsbSpeed) -> u32 {
        match speed {
            UsbSpeed::Low => SLOT_SPEED_LS,
            UsbSpeed::Full => SLOT_SPEED_FS,
            UsbSpeed::High => SLOT_SPEED_HS,
            UsbSpeed::Super => SLOT_SPEED_SS,
        }
    }

    fn speed_name(speed: UsbSpeed) -> &'static str {
        match speed {
            UsbSpeed::Low => "Low-Speed",
            UsbSpeed::Full => "Full-Speed",
            UsbSpeed::High => "High-Speed",
            UsbSpeed::Super => "SuperSpeed",
        }
    }

    fn hub_port_speed(hub_speed: UsbSpeed, port_status: u16) -> Result<UsbSpeed, ()> {
        if hub_speed == UsbSpeed::Super
            || (port_status & USB_PORT_STAT_SPEED_MASK) == USB_PORT_STAT_SUPER_SPEED
        {
            return Ok(UsbSpeed::Super);
        }
        Ok(match port_status & USB_PORT_STAT_SPEED_MASK {
            USB_PORT_STAT_LOW_SPEED => UsbSpeed::Low,
            USB_PORT_STAT_HIGH_SPEED => UsbSpeed::High,
            _ => UsbSpeed::Full,
        })
    }

    fn max_ports(value: u32) -> u32 {
        value << MAX_PORTS_SHIFT
    }

    fn tt_slot(value: u32) -> u32 {
        value
    }

    fn tt_port(value: u32) -> u32 {
        value << TT_PORT_SHIFT
    }

    fn tt_think_time(value: u32) -> u32 {
        value << TT_THINK_TIME_SHIFT
    }

    fn ep_type(value: u32) -> u32 {
        value << EP_TYPE_SHIFT
    }

    fn max_burst(value: u32) -> u32 {
        value << MAX_BURST_SHIFT
    }

    fn max_packet(value: u16) -> u32 {
        (value as u32) << MAX_PACKET_SHIFT
    }

    fn error_count(value: u32) -> u32 {
        value << ERROR_COUNT_SHIFT
    }

    fn trb_type(value: u32) -> u32 {
        value << TRB_TYPE_SHIFT
    }

    fn trb_field_to_type(field: u32) -> u32 {
        (field & TRB_TYPE_MASK) >> TRB_TYPE_SHIFT
    }

    fn trb_len(length: usize) -> u32 {
        (length as u32) & 0x1_FFFF
    }

    fn trb_td_size(value: usize) -> u32 {
        min(value as u32, 31) << 17
    }

    fn transfer_trb_remainder(
        transferred_before: usize,
        current_trb_len: usize,
        total_len: usize,
        max_packet: usize,
    ) -> usize {
        if current_trb_len == total_len
            || max_packet == 0
            || transferred_before + current_trb_len >= total_len
        {
            return 0;
        }
        let total_packets = total_len.div_ceil(max_packet);
        total_packets.saturating_sub((transferred_before + current_trb_len) / max_packet)
    }

    fn event_residue(transfer_len_field: u32) -> u32 {
        transfer_len_field & 0x00FF_FFFF
    }

    fn completion_code(status_field: u32) -> u32 {
        (status_field >> 24) & 0xFF
    }

    fn deadline(timeout_us: u64) -> u64 {
        let freq = asm::get_cntfrq_el0();
        let ticks = if freq == 0 {
            timeout_us
        } else {
            (timeout_us * freq).div_ceil(1_000_000)
        };
        asm::get_cntpct_el0().wrapping_add(ticks)
    }

    fn deadline_passed(deadline: u64) -> bool {
        asm::get_cntpct_el0().wrapping_sub(deadline) < (1u64 << 63)
    }

    fn delay_us(duration_us: u64) {
        let deadline = Self::deadline(duration_us);
        while !Self::deadline_passed(deadline) {
            core::hint::spin_loop();
        }
    }

    fn delay_ms(duration_ms: u16) {
        Self::delay_us(duration_ms as u64 * 1_000);
    }

    fn needs_bounce_buffer(buffer_address: usize, length: usize) -> bool {
        let line_size = asm::get_dcache_line_size();
        let mask = line_size - 1;
        (buffer_address & mask) != 0 || (length & mask) != 0
    }
}

impl DmaRegion {
    fn new(size: usize, align_order: usize) -> Result<Self, ()> {
        let pages = size.div_ceil(4096).max(1);
        /* The xHCI controller DMAs to/from this region; the BCM2711 PCIe
         * inbound window only reaches RAM below 4 GiB (see
         * crate::allocate_dma_pages). */
        let address = crate::allocate_dma_pages(pages, align_order).map_err(|_| ())?;
        unsafe { write_bytes(address as *mut u8, 0, pages * 4096) };
        let region = Self {
            address,
            size,
            pages,
        };
        /*
         * Flush the zero-fill to RAM immediately. The xHCI controller reads
         * this memory via DMA (bypassing the CPU cache), so any caller that
         * forgets to (or only partially, e.g. a single TRB's worth of) clean
         * this region before the controller's next doorbell/register write
         * risks the controller observing stale/garbage cache-line contents
         * instead of the zeroed buffer -- this was intermittently causing a
         * Host System Error (USBSTS HSE) right after starting the
         * controller, since the command ring/event ring segments were zeroed
         * here but never explicitly flushed in bulk before CRCR/ERSTBA were
         * programmed.
         */
        region.clean();
        Ok(region)
    }

    fn clean(&self) {
        unsafe { asm::clean_dcache_range(self.address, self.pages * 4096) };
    }

    fn clean_range(&self, offset: usize, size: usize) {
        unsafe { asm::clean_dcache_range(self.address + offset, size) };
    }

    fn invalidate(&self) {
        unsafe { asm::invalidate_dcache_range(self.address, self.pages * 4096) };
    }

    fn invalidate_range(&self, offset: usize, size: usize) {
        unsafe { asm::invalidate_dcache_range(self.address + offset, size) };
    }
}

impl TrbRing {
    fn new(link_trb: bool) -> Result<Self, ()> {
        let region = DmaRegion::new(TRBS_PER_SEGMENT * 16, 12)?;
        let ring = Self {
            region,
            enqueue_index: 0,
            cycle_state: 1,
        };
        if link_trb {
            let link = (ring.region.address + LINK_TRB_INDEX * 16) as *mut u32;
            let link_bus = to_bus(ring.region.address);
            unsafe {
                write_volatile(link.add(0), link_bus as u32);
                write_volatile(link.add(1), (link_bus >> 32) as u32);
                write_volatile(link.add(2), 0);
                write_volatile(link.add(3), Xhci::trb_type(TRB_LINK) | LINK_TOGGLE);
            }
            ring.region.clean_range(LINK_TRB_INDEX * 16, 16);
        }
        Ok(ring)
    }

    fn advance_enqueue(&mut self) {
        self.enqueue_index += 1;
        if self.enqueue_index == LINK_TRB_INDEX {
            let field3 = self.region.address + LINK_TRB_INDEX * 16 + 12;
            let current = unsafe { read_volatile(field3 as *const u32) };
            unsafe { write_volatile(field3 as *mut u32, current ^ TRB_CYCLE) };
            self.region.clean_range(LINK_TRB_INDEX * 16, 16);
            self.enqueue_index = 0;
            self.cycle_state ^= 1;
        }
    }
}

impl EventRing {
    fn new() -> Result<Self, ()> {
        Ok(Self {
            region: DmaRegion::new(TRBS_PER_SEGMENT * 16, 12)?,
            dequeue_index: 0,
            cycle_state: 1,
        })
    }
}

impl BounceBuffer {
    fn new(size: usize) -> Result<Self, ()> {
        Ok(Self {
            region: DmaRegion::new(size, 12)?,
        })
    }

    fn address(&self) -> usize {
        self.region.address
    }

    fn copy_from(&mut self, source_address: usize, size: usize) {
        unsafe {
            copy_nonoverlapping(
                source_address as *const u8,
                self.region.address as *mut u8,
                size,
            )
        };
    }

    fn copy_to(&self, destination_address: usize, size: usize) {
        unsafe {
            copy_nonoverlapping(
                self.region.address as *const u8,
                destination_address as *mut u8,
                size,
            )
        };
    }
}

impl Drop for BounceBuffer {
    fn drop(&mut self) {
        crate::free_pages(self.region.address, self.region.pages);
    }
}
