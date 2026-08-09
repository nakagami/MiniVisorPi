//!
//! Virtio-Console MMIO Driver
//!
//! Presents an emulated, legacy Virtio-Console device (2 VirtQueues:
//! RX=0 host->guest, TX=1 guest->host) to the guest. Unlike the emulated
//! PL011, which traps to EL2 twice per *character* (FR read + DR write),
//! console output here is batched in virtqueue buffers: the guest notifies
//! once per batch, which under TCG (where every MMIO trap is emulated in
//! software) cuts `ls -l /bin` console output from ~2 traps/char to ~1
//! trap/line. Guest console=`hvc0`.
//!

use crate::drivers::virtio::*;
use crate::mmio::virtio_net::QueueState;
use crate::vm::*;

use core::ptr::write_volatile;

/// Guest-visible virtual interrupt used to signal this device. Must match
/// the `interrupts` property of the `virtio_mmio@a000400` DTB node
/// (SPI 0x12/18 -> INTID 32 + 18 = 50).
const VIRTIO_CONSOLE_INT_ID: u32 = 50;
/// Number of descriptors reported to the guest for both VirtQueues
const GUEST_QUEUE_SIZE: usize = 32;
/// virtio-console device ID
const VIRTIO_ID_CONSOLE: u32 = 3;

const QUEUE_INDEX_RX: usize = 0;
const QUEUE_INDEX_TX: usize = 1;

/// Bytes of guest input held when the guest has no RX buffer posted yet.
const RX_STAGING_SIZE: usize = 256;

/// Diagnostic counters (see the `stat` console command).
pub static VCONSOLE_TX_BUFS: core::sync::atomic::AtomicU64 =
    core::sync::atomic::AtomicU64::new(0);
pub static VCONSOLE_TX_BYTES: core::sync::atomic::AtomicU64 =
    core::sync::atomic::AtomicU64::new(0);

pub struct VirtioConsoleMmio {
    interrupt_status: u32,
    status: u32,
    page_size: usize,
    queue_sel: usize,
    device_features_sel: u32,
    queues: [QueueState; 2],
    /* Input bytes received from the physical UART while the guest had no
     * RX buffer posted; flushed as soon as a buffer appears. */
    rx_staging: [u8; RX_STAGING_SIZE],
    rx_staging_len: usize,
}

impl VirtioConsoleMmio {
    pub fn new() -> Self {
        Self {
            interrupt_status: 0,
            status: 0,
            page_size: 1 << 12,
            queue_sel: 0,
            device_features_sel: 0,
            queues: [QueueState::new(), QueueState::new()],
            rx_staging: [0; RX_STAGING_SIZE],
            rx_staging_len: 0,
        }
    }

    fn current_queue(&mut self) -> &mut QueueState {
        &mut self.queues[self.queue_sel]
    }

    /// Feeds a byte received from the physical UART into the guest's RX
    /// queue (or into the staging buffer if no RX buffer is posted yet).
    pub fn push_rx(&mut self, data: u8, vm: &VM) {
        if self.rx_staging_len < RX_STAGING_SIZE {
            self.rx_staging[self.rx_staging_len] = data;
            self.rx_staging_len += 1;
        }
        self.flush_rx(vm);
    }

    /// Moves staged input bytes into guest-posted RX buffers, one byte per
    /// buffer (console input is human-speed, so per-byte used-ring entries
    /// and interrupts are fine and keep the hvc driver's read path simple).
    fn flush_rx(&mut self, vm: &VM) {
        let mut injected = false;
        while self.rx_staging_len > 0 {
            let rx = &mut self.queues[QUEUE_INDEX_RX];
            let Some(id) = rx.get_next_avail_id() else {
                break;
            };
            let Some(descriptor_id) = rx.get_descriptor_id(id) else {
                break;
            };
            let Some(descriptor) = rx.get_descriptor(descriptor_id) else {
                break;
            };
            if descriptor.length == 0 {
                break;
            }
            let Some(address) = vm.get_physical_address(descriptor.address as usize) else {
                break;
            };
            let data = self.rx_staging[0];
            unsafe { write_volatile(address as *mut u8, data) };
            self.rx_staging.copy_within(1..self.rx_staging_len, 0);
            self.rx_staging_len -= 1;
            self.queues[QUEUE_INDEX_RX].write_used(descriptor_id, 1);
            injected = true;
        }
        if injected {
            self.interrupt_status |= 1;
            vm.get_gic_distributor_mmio()
                .lock()
                .trigger_interrupt(VIRTIO_CONSOLE_INT_ID, None);
        }
    }

    /// Handles guest console output (QUEUE_NOTIFY on the TX queue): prints
    /// each posted buffer to the physical console.
    fn process_tx(&mut self, vm: &VM) {
        let mut handled = false;
        while let Some(id) = self.queues[QUEUE_INDEX_TX].get_next_avail_id() {
            let Some(descriptor_id) = self.queues[QUEUE_INDEX_TX].get_descriptor_id(id) else {
                println!("Virtio-Console: failed to get the next TX descriptor id");
                return;
            };
            /* A virtio-console TX request is a single descriptor holding the
             * raw characters (no header, unlike virtio-blk/net). */
            let Some(descriptor) = self.queues[QUEUE_INDEX_TX].get_descriptor(descriptor_id)
            else {
                println!("Virtio-Console: failed to get the TX descriptor");
                return;
            };
            let Some(address) = vm.get_physical_address(descriptor.address as usize) else {
                println!("Virtio-Console: failed to translate the TX buffer address");
                return;
            };
            let bytes =
                unsafe { core::slice::from_raw_parts(address as *const u8, descriptor.length as usize) };
            match core::str::from_utf8(bytes) {
                Ok(s) => print!("{s}"),
                Err(_) => {
                    for &b in bytes {
                        print!("{}", b as char);
                    }
                }
            }
            {
                use core::sync::atomic::Ordering;
                VCONSOLE_TX_BUFS.fetch_add(1, Ordering::Relaxed);
                VCONSOLE_TX_BYTES.fetch_add(descriptor.length as u64, Ordering::Relaxed);
            }
            self.queues[QUEUE_INDEX_TX].write_used(descriptor_id, descriptor.length);
            handled = true;
        }
        if handled {
            self.interrupt_status |= 1;
            vm.get_gic_distributor_mmio()
                .lock()
                .trigger_interrupt(VIRTIO_CONSOLE_INT_ID, None);
        }
    }
}

impl MmioHandler for VirtioConsoleMmio {
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
                value = VIRTIO_ID_CONSOLE as u64;
            }
            VIRTIO_MMIO_VENDOR_ID => {
                value = 0x554d4551;
            }
            VIRTIO_MMIO_DEVICE_FEATURES => {
                /* No features: no VIRTIO_CONSOLE_F_MULTIPORT etc., so the
                 * guest uses port 0 only and never reads the config space
                 * (which is therefore left all-zero). */
                value = 0;
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
            _ => { /* Unimplemented (includes the all-zero config space) */ }
        }
        Ok(value)
    }

    fn write(&mut self, offset: usize, _access_width: u64, value: u64) -> Result<(), ()> {
        match offset {
            VIRTIO_MMIO_DEVICE_FEATURES_SEL => {
                self.device_features_sel = value as u32;
            }
            VIRTIO_MMIO_DRIVER_FEATURES_SEL | VIRTIO_MMIO_DRIVER_FEATURES => {
                /* No features are advertised; nothing to negotiate */
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
                } else {
                    println!("Virtio-Console: QUEUE_PFN address translation failed");
                }
            }
            VIRTIO_MMIO_QUEUE_NOTIFY => {
                let vm = get_current_vm();
                if value as usize == QUEUE_INDEX_TX {
                    self.process_tx(&vm);
                } else if value as usize == QUEUE_INDEX_RX {
                    /* The guest just posted fresh RX buffers: deliver any
                     * staged input bytes. */
                    self.flush_rx(&vm);
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
                    self.rx_staging_len = 0;
                } else {
                    self.status = value as u32;
                }
            }
            _ => { /* Unimplemented */ }
        }
        Ok(())
    }
}

unsafe impl core::marker::Send for VirtioConsoleMmio {}
