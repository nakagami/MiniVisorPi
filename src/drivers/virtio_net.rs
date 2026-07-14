//!
//! Virtio-Net implementation (physical device driver)
//!
//! Drives the real QEMU `virtio-net-device` so the hypervisor can forward
//! Ethernet frames between the guest's emulated Virtio-Net MMIO device
//! (see `crate::mmio::virtio_net`) and the outside world.
//!

use crate::drivers::virtio::*;

/// Host has given MAC address
const VIRTIO_NET_F_MAC: u32 = 1 << 5;

/// Number of descriptors used for both the RX(0) and TX(1) VirtQueues.
/// (RX keeps one buffer posted per descriptor, TX uses 2 descriptors per packet)
const NUMBER_OF_DESCRIPTORS: usize = 8;
/// 1500(MTU) + 14(Ethernet header) + 12(virtio_net_hdr, rounded up)
pub const VIRTIO_NET_RX_BUFFER_SIZE: usize = 1526;
/// Legacy virtio-net header prepended to every packet
const VIRTIO_NET_HDR_SIZE: usize = 10;

/// A single VirtQueue (RX or TX), sized `NUMBER_OF_DESCRIPTORS`.
struct VirtQueue {
    descriptors: *mut [VirtQueueDesc; NUMBER_OF_DESCRIPTORS],
    avail: *mut VirtQueueAvail,
    used: *mut VirtQueueUsed,
    last_used_idx: u16,
    free_bitmap: [u8; NUMBER_OF_DESCRIPTORS / (u8::BITS as usize)],
}

impl VirtQueue {
    fn setup(base_address: usize, queue_index: u32) -> Result<Self, ()> {
        VirtioNet::write_register(base_address, VIRTIO_MMIO_QUEUE_SEL, queue_index);
        let queue_max = VirtioNet::read_register(base_address, VIRTIO_MMIO_QUEUE_NUM_MAX);
        if (queue_max as usize) < NUMBER_OF_DESCRIPTORS {
            println!("Virtio-Net Queue({queue_index}) Size is invalid: {queue_max}");
            return Err(());
        }
        VirtioNet::write_register(
            base_address,
            VIRTIO_MMIO_QUEUE_NUM,
            NUMBER_OF_DESCRIPTORS as u32,
        );

        let number_of_pages_queue = (((size_of::<VirtQueueDesc>() * NUMBER_OF_DESCRIPTORS
            + size_of::<VirtQueueAvail>())
            >> VIRTIO_PAGE_SHIFT)
            + 1)
            + ((size_of::<VirtQueueUsed>() >> VIRTIO_PAGE_SHIFT) + 1);
        let queue = crate::allocate_pages(number_of_pages_queue, VIRTIO_PAGE_SHIFT)
            .expect("Failed to allocate virtio-net queue");
        unsafe {
            core::ptr::write_bytes(queue as *mut u8, 0, number_of_pages_queue << VIRTIO_PAGE_SHIFT)
        };
        VirtioNet::write_register(
            base_address,
            VIRTIO_MMIO_QUEUE_PFN,
            (queue >> VIRTIO_PAGE_SHIFT) as u32,
        );

        let descriptor_table = queue;
        let available_ring = descriptor_table + size_of::<VirtQueueDesc>() * NUMBER_OF_DESCRIPTORS;
        let used_ring = ((available_ring + size_of::<VirtQueueAvail>() - 1)
            & !(VIRTIO_PAGE_SIZE - 1))
            + VIRTIO_PAGE_SIZE;
        Ok(Self {
            descriptors: descriptor_table as *mut _,
            avail: available_ring as *mut _,
            used: used_ring as *mut _,
            last_used_idx: 0,
            free_bitmap: [u8::MAX; NUMBER_OF_DESCRIPTORS / (u8::BITS as usize)],
        })
    }

    fn allocate_descriptor(&mut self) -> Option<(u16, &'static mut VirtQueueDesc)> {
        for (byte, c) in self.free_bitmap.iter_mut().enumerate() {
            for bit in 0..(u8::BITS as usize) {
                if (*c & (1 << bit)) != 0 {
                    *c &= !(1 << bit);
                    let index = (byte * u8::BITS as usize) + bit;
                    return Some((index as u16, &mut unsafe { &mut *self.descriptors }[index]));
                }
            }
        }
        None
    }

    fn free_descriptor(&mut self, index: u16) {
        assert!((index as usize) < NUMBER_OF_DESCRIPTORS);
        self.free_bitmap[(index as usize) / (u8::BITS as usize)] |=
            1 << (index as usize % (u8::BITS as usize));
    }
}

pub struct VirtioNet {
    base_address: usize,
    mac: [u8; 6],
    rx: VirtQueue,
    tx: VirtQueue,
    rx_buffers: usize,
}

impl VirtioNet {
    fn read_register(base_address: usize, offset: usize) -> u32 {
        unsafe { core::ptr::read_volatile((base_address + offset) as *const u32) }
    }

    fn write_register(base_address: usize, offset: usize, data: u32) {
        unsafe { core::ptr::write_volatile((base_address + offset) as *mut u32, data) }
    }

    pub fn new(base_address: usize) -> Result<Self, ()> {
        if Self::read_register(base_address, VIRTIO_MMIO_MAGIC) != VIRTIO_MMIO_MAGIC_VALUE {
            return Err(());
        }
        if Self::read_register(base_address, VIRTIO_MMIO_VERSION) != 1 {
            return Err(());
        }
        if Self::read_register(base_address, VIRTIO_MMIO_DEVICE_ID) != VIRTIO_ID_NET
            || Self::read_register(base_address, VIRTIO_MMIO_VENDOR_ID) != 0x554d4551
        {
            return Err(());
        }

        /* Reset the device */
        Self::write_register(base_address, VIRTIO_MMIO_STATUS, 0);
        Self::write_register(base_address, VIRTIO_MMIO_STATUS, VIRTIO_DEVICE_STATUS_ACKNOWLEDGE);
        Self::write_register(
            base_address,
            VIRTIO_MMIO_STATUS,
            Self::read_register(base_address, VIRTIO_MMIO_STATUS) | VIRTIO_DEVICE_STATUS_DRIVER,
        );

        /* Only negotiate VIRTIO_NET_F_MAC, to keep the driver simple */
        let device_features = Self::read_register(base_address, VIRTIO_MMIO_DEVICE_FEATURES);
        let driver_features = device_features & VIRTIO_NET_F_MAC;
        Self::write_register(base_address, VIRTIO_MMIO_DRIVER_FEATURES, driver_features);
        Self::write_register(
            base_address,
            VIRTIO_MMIO_STATUS,
            Self::read_register(base_address, VIRTIO_MMIO_STATUS)
                | VIRTIO_DEVICE_STATUS_FEATURES_OK,
        );

        Self::write_register(
            base_address,
            VIRTIO_MMIO_GUEST_PAGE_SIZE,
            VIRTIO_PAGE_SIZE as u32,
        );

        /* Set up the RX(0) and TX(1) VirtQueues */
        let rx = VirtQueue::setup(base_address, 0)?;
        let tx = VirtQueue::setup(base_address, 1)?;

        Self::write_register(
            base_address,
            VIRTIO_MMIO_STATUS,
            Self::read_register(base_address, VIRTIO_MMIO_STATUS) | VIRTIO_DEVICE_STATUS_DRIVER_OK,
        );

        /* Read the MAC address if the device supports it */
        let mut mac = [0u8; 6];
        if (driver_features & VIRTIO_NET_F_MAC) != 0 {
            let low = Self::read_register(base_address, VIRTIO_CONFIG_OFFSET);
            let high = Self::read_register(base_address, VIRTIO_CONFIG_OFFSET + 4);
            mac = [
                low as u8,
                (low >> 8) as u8,
                (low >> 16) as u8,
                (low >> 24) as u8,
                high as u8,
                (high >> 8) as u8,
            ];
        }

        let rx_buffers = crate::allocate_pages(
            (NUMBER_OF_DESCRIPTORS * VIRTIO_NET_RX_BUFFER_SIZE).div_ceil(VIRTIO_PAGE_SIZE),
            VIRTIO_PAGE_SHIFT,
        )
        .expect("Failed to allocate RX buffers for Virtio-Net");

        let mut net = Self {
            base_address,
            mac,
            rx,
            tx,
            rx_buffers,
        };
        net.post_all_rx_buffers();
        Ok(net)
    }

    pub fn get_mac_address(&self) -> [u8; 6] {
        self.mac
    }

    fn post_all_rx_buffers(&mut self) {
        for i in 0..NUMBER_OF_DESCRIPTORS {
            let Some((idx, descriptor)) = self.rx.allocate_descriptor() else {
                break;
            };
            descriptor.address = (self.rx_buffers + i * VIRTIO_NET_RX_BUFFER_SIZE) as u64;
            descriptor.length = VIRTIO_NET_RX_BUFFER_SIZE as u32;
            descriptor.flags = VIRT_QUEUE_DESC_FLAGS_WRITE;
            descriptor.next = 0;

            let avail_ring = unsafe { &mut *self.rx.avail };
            let avail_idx = avail_ring.idx as usize;
            avail_ring.ring[avail_idx % NUMBER_OF_DESCRIPTORS] = idx;
            avail_ring.idx += 1;
        }
        Self::write_register(self.base_address, VIRTIO_MMIO_QUEUE_NOTIFY, 0);
    }

    fn repost_rx_buffer(&mut self, descriptor_id: u16) {
        {
            let descriptor = &mut unsafe { &mut *self.rx.descriptors }[descriptor_id as usize];
            descriptor.length = VIRTIO_NET_RX_BUFFER_SIZE as u32;
            descriptor.flags = VIRT_QUEUE_DESC_FLAGS_WRITE;
        }
        let avail_ring = unsafe { &mut *self.rx.avail };
        let avail_idx = avail_ring.idx as usize;
        avail_ring.ring[avail_idx % NUMBER_OF_DESCRIPTORS] = descriptor_id;
        avail_ring.idx += 1;
        Self::write_register(self.base_address, VIRTIO_MMIO_QUEUE_NOTIFY, 0);
    }

    /// Copies at most one received Ethernet frame (without the virtio_net_hdr)
    /// into `buffer`, returning its length. Recycles the consumed RX buffer.
    pub fn poll_rx(&mut self, buffer: &mut [u8]) -> Option<usize> {
        /* Acknowledge the physical device's interrupt so it de-asserts the
         * (level-sensitive in QEMU's actual implementation) IRQ line; without
         * this the line stays asserted and the GIC keeps re-presenting it. */
        let interrupt_status = Self::read_register(self.base_address, VIRTIO_MMIO_INTERRUPT_STATUS);
        if interrupt_status != 0 {
            Self::write_register(self.base_address, VIRTIO_MMIO_INTERRUPT_ACK, interrupt_status);
        }

        let used_ring = unsafe { &*self.rx.used };
        if used_ring.idx == self.rx.last_used_idx {
            return None;
        }
        let element = unsafe {
            core::ptr::read_volatile(
                &used_ring.ring[(self.rx.last_used_idx as usize) % NUMBER_OF_DESCRIPTORS]
                    as *const VirtQueueUsedElement,
            )
        };
        self.rx.last_used_idx = self.rx.last_used_idx.wrapping_add(1);

        let descriptor_id = element.id as u16;
        let total_length = element.length as usize;
        let payload_length = total_length.saturating_sub(VIRTIO_NET_HDR_SIZE);
        let copy_length = payload_length.min(buffer.len());

        let source = (self.rx_buffers
            + (descriptor_id as usize) * VIRTIO_NET_RX_BUFFER_SIZE
            + VIRTIO_NET_HDR_SIZE) as *const u8;
        unsafe {
            core::ptr::copy_nonoverlapping(source, buffer.as_mut_ptr(), copy_length);
        }

        self.repost_rx_buffer(descriptor_id);
        Some(copy_length)
    }

    /// Synchronously transmits the Ethernet frame located at `buffer_address`
    /// (a physical address readable by the hypervisor).
    pub fn send(&mut self, buffer_address: usize, length: usize) -> Result<(), ()> {
        static HEADER: [u8; VIRTIO_NET_HDR_SIZE] = [0; VIRTIO_NET_HDR_SIZE];

        let Some((header_idx, header_descriptor)) = self.tx.allocate_descriptor() else {
            println!("Failed to allocate TX header descriptor");
            return Err(());
        };
        header_descriptor.address = HEADER.as_ptr() as usize as u64;
        header_descriptor.length = VIRTIO_NET_HDR_SIZE as u32;
        header_descriptor.flags = VIRT_QUEUE_DESC_FLAGS_NEXT;

        let Some((data_idx, data_descriptor)) = self.tx.allocate_descriptor() else {
            println!("Failed to allocate TX data descriptor");
            self.tx.free_descriptor(header_idx);
            return Err(());
        };
        data_descriptor.address = buffer_address as u64;
        data_descriptor.length = length as u32;
        data_descriptor.flags = 0;
        header_descriptor.next = data_idx;

        let avail_ring = unsafe { &mut *self.tx.avail };
        let idx = avail_ring.idx as usize;
        avail_ring.ring[idx % NUMBER_OF_DESCRIPTORS] = header_idx;
        avail_ring.idx += 1;

        Self::write_register(self.base_address, VIRTIO_MMIO_QUEUE_NOTIFY, 1);

        /* Spin wait for the transmission to complete */
        let target_idx = self.tx.last_used_idx.wrapping_add(1);
        loop {
            unsafe { crate::asm::invalidate_cache(self.tx.used as usize) };
            let used_ring = unsafe { &*self.tx.used };
            if used_ring.idx == target_idx {
                break;
            }
            core::hint::spin_loop();
        }
        self.tx.last_used_idx = target_idx;

        self.tx.free_descriptor(header_idx);
        self.tx.free_descriptor(data_idx);
        Ok(())
    }
}

unsafe impl core::marker::Send for VirtioNet {}
