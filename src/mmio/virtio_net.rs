//!
//! Virtio-Net MMIO Driver
//!
//! Presents an emulated, legacy Virtio-Net device (2 VirtQueues: RX=0, TX=1)
//! to the guest. Outgoing packets (TX) are forwarded synchronously to the
//! active physical backend (`crate::PHYSICAL_NET`) through `send()`.
//! Incoming packets (RX) are injected by `push_rx()`, called from the physical
//! backend polling/interrupt path (see `crate::handle_net_rx`).
//!

use crate::drivers::virtio::*;
use crate::vm::*;
use crate::PHYSICAL_NET;

use core::ptr::{null_mut, read_volatile, write_volatile};

/// Guest-visible virtual interrupt used to signal this device. Must exactly
/// match the `interrupts` property of the `virtio_mmio@a000200` DTB node
/// (SPI 0x11/17 -> INTID 32 + 17 = 49), since the guest OS (Linux) enables
/// and waits on this specific interrupt line (unlike u-boot's virtio-net
/// driver, which polls the used ring directly and never enables interrupts).
const VIRTIO_NET_INT_ID: u32 = 49;
/// Number of descriptors reported to the guest for both VirtQueues
const GUEST_QUEUE_SIZE: usize = 32;
/// Legacy virtio-net header size (VIRTIO_NET_F_MRG_RXBUF not negotiated)
const VIRTIO_NET_HDR_SIZE: usize = 10;
/// Host has given MAC address
const VIRTIO_NET_F_MAC: u32 = 1 << 5;

const QUEUE_INDEX_RX: usize = 0;
const QUEUE_INDEX_TX: usize = 1;

struct QueueState {
    queue_size: usize,
    descriptor: *mut VirtQueueDesc,
    avail_ring: *mut VirtQueueAvail,
    used_ring: *mut VirtQueueUsed,
    last_avail_id: u16,
    used_id: u16,
}

impl QueueState {
    const fn new() -> Self {
        Self {
            queue_size: 0,
            descriptor: null_mut(),
            avail_ring: null_mut(),
            used_ring: null_mut(),
            last_avail_id: 0,
            used_id: 0,
        }
    }
    fn get_descriptor(&self, id: u16) -> Option<VirtQueueDesc> {
        if !self.descriptor.is_null() {
            Some(unsafe {
                read_volatile(
                    (self.descriptor as usize + size_of::<VirtQueueDesc>() * (id as usize))
                        as *const VirtQueueDesc,
                )
            })
        } else {
            None
        }
    }

    fn get_descriptor_id(&self, id: u16) -> Option<u16> {
        if !self.avail_ring.is_null() {
            Some(unsafe {
                read_volatile(
                    (self.avail_ring as usize
                        + size_of::<u16>() * 2 /* flag + idx */
                        + size_of::<u16>() * (id as usize)) as *const u16,
                )
            })
        } else {
            None
        }
    }

    fn get_next_avail_id(&mut self) -> Option<u16> {
        if self.avail_ring.is_null() || self.queue_size == 0 {
            return None;
        }
        if self.last_avail_id == unsafe { &*self.avail_ring }.idx {
            return None;
        }
        let next = self.last_avail_id % (self.queue_size as u16);
        self.last_avail_id += 1;
        Some(next)
    }

    fn write_used(&mut self, id: u16, length: u32) {
        let used_id = self.used_id % (self.queue_size as u16);
        self.used_id += 1;
        unsafe {
            write_volatile(
                (self.used_ring as usize
                    + size_of::<u16>() * 2 /* flag + idx */
                    + size_of::<VirtQueueUsedElement>() * (used_id as usize))
                    as *mut VirtQueueUsedElement,
                VirtQueueUsedElement {
                    id: (id as u32),
                    length,
                },
            )
        };
        unsafe { &mut *self.used_ring }.idx = self.used_id;
    }

    fn recompute_rings(&mut self, page_size: usize) {
        self.avail_ring =
            ((self.descriptor as usize) + size_of::<VirtQueueDesc>() * self.queue_size) as *mut _;
        self.used_ring = ((((self.avail_ring as usize + size_of::<u16>() * (3 + self.queue_size))
            - 1)
            & !(page_size - 1))
            + page_size) as *mut _;
    }

    fn reset(&mut self) {
        self.queue_size = 0;
        self.descriptor = null_mut();
        self.avail_ring = null_mut();
        self.used_ring = null_mut();
        self.last_avail_id = 0;
        self.used_id = 0;
    }
}

pub struct VirtioNetMmio {
    mac: [u8; 6],
    interrupt_status: u32,
    status: u32,
    page_size: usize,
    queue_sel: usize,
    device_features_sel: u32,
    queues: [QueueState; 2],
}

impl VirtioNetMmio {
    pub fn new(mac: [u8; 6]) -> Self {
        Self {
            mac,
            interrupt_status: 0,
            status: 0,
            page_size: 1 << 12,
            queue_sel: 0,
            device_features_sel: 0,
            queues: [QueueState::new(), QueueState::new()],
        }
    }

    fn current_queue(&mut self) -> &mut QueueState {
        &mut self.queues[self.queue_sel]
    }

    /// Called by the physical Virtio-Net interrupt handler when a new
    /// Ethernet frame has arrived. Injects it into the guest's RX queue.
    pub fn push_rx(&mut self, data: &[u8]) {
        let rx = &mut self.queues[QUEUE_INDEX_RX];
        let Some(id) = rx.get_next_avail_id() else {
            /* No RX buffer posted by the guest yet: drop the packet */
            return;
        };
        let Some(descriptor_id) = rx.get_descriptor_id(id) else {
            return;
        };
        let Some(descriptor) = rx.get_descriptor(descriptor_id) else {
            return;
        };
        if (descriptor.length as usize) < VIRTIO_NET_HDR_SIZE + data.len() {
            println!("Virtio-Net RX buffer is too small");
            return;
        }
        let Some(address) = get_current_vm().get_physical_address(descriptor.address as usize)
        else {
            println!("Failed to translate the Virtio-Net RX buffer address");
            return;
        };
        unsafe {
            core::ptr::write_bytes(address as *mut u8, 0, VIRTIO_NET_HDR_SIZE);
            core::ptr::copy_nonoverlapping(
                data.as_ptr(),
                (address + VIRTIO_NET_HDR_SIZE) as *mut u8,
                data.len(),
            );
        }
        self.queues[QUEUE_INDEX_RX]
            .write_used(descriptor_id, (VIRTIO_NET_HDR_SIZE + data.len()) as u32);
        self.interrupt_status |= 1;
        println!("Virtio-Net RX: delivered {} bytes to guest", data.len());
        get_current_vm()
            .get_gic_distributor_mmio()
            .lock()
            .trigger_interrupt(VIRTIO_NET_INT_ID, None);
    }

    /// Handles a guest-issued transmission (QUEUE_NOTIFY on the TX queue).
    fn process_tx(&mut self) {
        while let Some(id) = self.queues[QUEUE_INDEX_TX].get_next_avail_id() {
            let Some(descriptor_id) = self.queues[QUEUE_INDEX_TX].get_descriptor_id(id) else {
                println!("Failed to get the next TX descriptor id");
                return;
            };
            let Some(header_descriptor) = self.queues[QUEUE_INDEX_TX].get_descriptor(descriptor_id)
            else {
                println!("Failed to get the TX header descriptor");
                return;
            };
            if (header_descriptor.flags & VIRT_QUEUE_DESC_FLAGS_NEXT) == 0 {
                println!("Invalid Virtio-Net TX request (no payload descriptor)");
                continue;
            }
            let Some(data_descriptor) =
                self.queues[QUEUE_INDEX_TX].get_descriptor(header_descriptor.next)
            else {
                println!("Failed to get the TX data descriptor");
                return;
            };
            let total_length = header_descriptor.length + data_descriptor.length;
            if let Some(address) =
                get_current_vm().get_physical_address(data_descriptor.address as usize)
            {
                let mut net = PHYSICAL_NET.lock();
                if let Some(net) = net.as_mut() {
                    match net.send(address, data_descriptor.length as usize) {
                        Ok(()) => {
                            println!(
                                "Virtio-Net TX: sent {} bytes",
                                data_descriptor.length
                            );
                        }
                        Err(()) => {
                            println!(
                                "Virtio-Net TX: physical send() failed ({} bytes)",
                                data_descriptor.length
                            );
                        }
                    }
                }
            } else {
                println!("Failed to translate the Virtio-Net TX buffer address");
            }
            self.queues[QUEUE_INDEX_TX].write_used(descriptor_id, total_length);
        }
        self.interrupt_status |= 1;
        get_current_vm()
            .get_gic_distributor_mmio()
            .lock()
            .trigger_interrupt(VIRTIO_NET_INT_ID, None);
    }
}

impl MmioHandler for VirtioNetMmio {
    fn read(&mut self, offset: usize, _access_width: u64) -> Result<u64, ()> {
        let mut value = 0u64;
        match offset {
            VIRTIO_MMIO_MAGIC => {
                value = VIRTIO_MMIO_MAGIC_VALUE as u64;
            }
            VIRTIO_MMIO_VERSION => {
                value = 0x01;
            }
            VIRTIO_MMIO_DEVICE_ID => {
                value = VIRTIO_ID_NET as u64;
            }
            VIRTIO_MMIO_VENDOR_ID => {
                value = 0x554d4551;
            }
            VIRTIO_MMIO_DEVICE_FEATURES => {
                value = if self.device_features_sel == 0 {
                    VIRTIO_NET_F_MAC as u64
                } else {
                    0
                };
            }
            VIRTIO_MMIO_QUEUE_NUM_MAX => {
                value = GUEST_QUEUE_SIZE as u64;
            }
            VIRTIO_MMIO_QUEUE_PFN => {
                let queue = &self.queues[self.queue_sel];
                value = (queue.descriptor as usize / self.page_size) as u64;
            }
            VIRTIO_MMIO_INTERRUPT_STATUS => {
                value = self.interrupt_status as u64;
            }
            VIRTIO_MMIO_STATUS => {
                value = self.status as u64;
            }
            _ if offset >= VIRTIO_CONFIG_OFFSET => {
                let config_offset = offset - VIRTIO_CONFIG_OFFSET;
                if (0..6).contains(&config_offset) {
                    value = self.mac[config_offset] as u64;
                } else if config_offset == 6 {
                    /* virtio_net_config.status: link is always up */
                    value = 1;
                }
            }
            _ => { /* Unimplemented */ }
        }
        Ok(value)
    }

    fn write(&mut self, offset: usize, _access_width: u64, value: u64) -> Result<(), ()> {
        match offset {
            VIRTIO_MMIO_DEVICE_FEATURES_SEL => {
                self.device_features_sel = value as u32;
            }
            VIRTIO_MMIO_DRIVER_FEATURES_SEL | VIRTIO_MMIO_DRIVER_FEATURES => {
                /* Only VIRTIO_NET_F_MAC is advertised; nothing to negotiate */
            }
            VIRTIO_MMIO_GUEST_PAGE_SIZE => {
                self.page_size = value as usize;
            }
            VIRTIO_MMIO_QUEUE_SEL => {
                if value < 2 {
                    self.queue_sel = value as usize;
                }
            }
            VIRTIO_MMIO_QUEUE_NUM => {
                let page_size = self.page_size;
                let queue = self.current_queue();
                queue.queue_size = value as usize;
                queue.recompute_rings(page_size);
            }
            VIRTIO_MMIO_QUEUE_PFN => {
                let page_size = self.page_size;
                if let Some(address) =
                    get_current_vm().get_physical_address((value as usize) * page_size)
                {
                    let queue = self.current_queue();
                    queue.descriptor = address as *mut _;
                    queue.recompute_rings(page_size);
                }
            }
            VIRTIO_MMIO_QUEUE_NOTIFY => {
                if value as usize == QUEUE_INDEX_TX {
                    self.process_tx();
                }
            }
            VIRTIO_MMIO_INTERRUPT_ACK => {
                self.interrupt_status &= !(value as u32);
            }
            VIRTIO_MMIO_STATUS => {
                if value == 0 {
                    self.page_size = 1 << 12;
                    self.interrupt_status = 0;
                    self.status = 0;
                    self.queue_sel = 0;
                    self.device_features_sel = 0;
                    for queue in &mut self.queues {
                        queue.reset();
                    }
                } else {
                    self.status = value as u32;
                }
            }
            _ => { /* Unimplemented */ }
        }
        Ok(())
    }
}

unsafe impl core::marker::Send for VirtioNetMmio {}
