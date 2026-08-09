//!
//! Console
//!

use core::str::SplitWhitespace;
use core::sync::atomic::Ordering;

pub struct Console {
    buffer: [u8; Self::BUFFER_SIZE],
    buffer_pointer: usize,
    ignore_lf: bool,
}

impl Console {
    const BUFFER_SIZE: usize = 64;
    #[allow(clippy::type_complexity)]
    const COMMAND_LIST: [(&str, fn(SplitWhitespace) -> bool); 6] = [
        ("boot", Self::boot_vm),
        ("switch", Self::switch_vm),
        ("spawn", Self::spawn_vm),
        ("echo", Self::echo),
        ("poweroff", Self::power_off),
        ("stat", Self::stat),
    ];

    pub const fn new() -> Self {
        Self {
            buffer: [0; Self::BUFFER_SIZE],
            buffer_pointer: 0,
            ignore_lf: false,
        }
    }

    pub fn write(&mut self, data: u8) {
        if data == b'\r' || (data == b'\n' && !self.ignore_lf) {
            self.ignore_lf = data == b'\r';
            println!();
            self.exec_command();
            return;
        }
        if data.is_ascii_control() || self.buffer_pointer == Self::BUFFER_SIZE {
            return;
        }
        self.buffer[self.buffer_pointer] = data;
        self.buffer_pointer += 1;
        print!("{}", data as char);
    }

    pub fn exec_command(&mut self) {
        if self.buffer_pointer == 0 {
            self.reset_buffer();
            return;
        }
        let Ok(input) = core::str::from_utf8(&self.buffer[0..self.buffer_pointer]) else {
            println!("Failed to parse the input");
            self.reset_buffer();
            return;
        };
        let mut command_list = input.split_whitespace();
        let Some(command) = command_list.next() else {
            self.reset_buffer();
            return;
        };
        if let Some((_, f)) = Self::COMMAND_LIST.iter().find(|&&(c, _)| c == command) {
            if f(command_list) {
                self.reset_buffer();
            } else {
                /* Automatically deactivate the console */
                crate::IS_CONSOLE_ACTIVE.fetch_xor(true, Ordering::Relaxed);
            }
        } else {
            println!("{} is not defined", command);
            self.reset_buffer();
        }
    }

    pub fn reset_buffer(&mut self) {
        self.buffer_pointer = 0;
        print!("Command>");
    }

    /* Implementation of each command */

    pub fn echo(list: SplitWhitespace) -> bool {
        for arg in list {
            print!("{} ", arg);
        }
        println!();
        true
    }

    pub fn power_off(_: SplitWhitespace) -> bool {
        println!("The host machine will shutdown!");
        crate::psci::system_off()
    }

    /// Dumps guest/physical I/O diagnostic counters (requests, bytes, FAT
    /// chain walk steps, and CNTPCT cycle totals/maxima).
    pub fn stat(_: SplitWhitespace) -> bool {
        use core::sync::atomic::Ordering;
        let freq = crate::asm::get_cntfrq_el0();
        let reqs = crate::mmio::virtio_blk::VBLK_REQUESTS.load(Ordering::Relaxed);
        let bytes = crate::mmio::virtio_blk::VBLK_BYTES.load(Ordering::Relaxed);
        let total = crate::mmio::virtio_blk::VBLK_CYCLES_TOTAL.load(Ordering::Relaxed);
        let max = crate::mmio::virtio_blk::VBLK_CYCLES_MAX.load(Ordering::Relaxed);
        println!(
            "guest virtio-blk: requests={} bytes={:#x} total={}ms max={}ms",
            reqs,
            bytes,
            total / (freq / 1000),
            max / (freq / 1000)
        );
        let pcount = crate::drivers::virtio_blk::PHYS_IO_COUNT.load(Ordering::Relaxed);
        let ptotal = crate::drivers::virtio_blk::PHYS_IO_CYCLES_TOTAL.load(Ordering::Relaxed);
        let pmax = crate::drivers::virtio_blk::PHYS_IO_CYCLES_MAX.load(Ordering::Relaxed);
        println!(
            "physical blk io: count={} total={}ms max={}ms",
            pcount,
            ptotal / (freq / 1000),
            pmax / (freq / 1000)
        );
        println!(
            "fat walk steps: {}",
            crate::fat32::FAT_WALK_STEPS.load(Ordering::Relaxed)
        );
        let vwrites = crate::mmio::pl011::VUART_DR_WRITES.load(Ordering::Relaxed);
        let vcycles = crate::mmio::pl011::VUART_DR_WRITE_CYCLES.load(Ordering::Relaxed);
        println!(
            "guest uart tx: chars={} total={}ms avg={}us",
            vwrites,
            vcycles / (freq / 1000),
            if vwrites > 0 { vcycles * 1_000_000 / vwrites / freq } else { 0 }
        );
        let pputc = crate::drivers::pl011::PHYS_PUTC_COUNT.load(Ordering::Relaxed);
        let pcycles = crate::drivers::pl011::PHYS_PUTC_CYCLES.load(Ordering::Relaxed);
        println!(
            "physical uart putc: count={} total={}ms avg={}us",
            pputc,
            pcycles / (freq / 1000),
            if pputc > 0 { pcycles * 1_000_000 / pputc / freq } else { 0 }
        );
        let cbufs = crate::mmio::virtio_console::VCONSOLE_TX_BUFS.load(Ordering::Relaxed);
        let cbytes = crate::mmio::virtio_console::VCONSOLE_TX_BYTES.load(Ordering::Relaxed);
        println!(
            "guest virtio-console tx: buffers={} bytes={:#x} avg_bytes_per_buf={}",
            cbufs,
            cbytes,
            if cbufs > 0 { cbytes / cbufs } else { 0 }
        );
        println!(
            "wfi polls: {}",
            crate::exception::WFI_POLL_COUNT.load(Ordering::Relaxed)
        );
        println!(
            "gich_lr overflows (retried): {}",
            crate::vgic::LR_OVERFLOW_COUNT.load(Ordering::Relaxed)
        );
        true
    }

    pub fn boot_vm(_: SplitWhitespace) -> bool {
        if crate::launch_cpu() {
            /* The Active VM switches automatically */
            println!("Booted a new VM");
            false
        } else {
            println!("Failed to boot a VM");
            true
        }
    }

    /// Queues a new VCPU on the *current* pCPU (the one whose UART
    /// interrupt/console-switch-key handling is driving this command),
    /// instead of `boot`'s new-physical-core path. It becomes active only
    /// once the currently running VCPU on this pCPU yields via WFI/WFE
    /// (see `vm::try_yield_to_next_vcpu`).
    pub fn spawn_vm(_: SplitWhitespace) -> bool {
        crate::spawn_vcpu_on_current_pcpu();
        println!("Queued a new VCPU on this pCPU");
        true
    }

    pub fn switch_vm(mut args: SplitWhitespace) -> bool {
        let Some(arg) = args.next() else {
            println!("Missing vm_id\nUsage: switch vm_id");
            return true;
        };
        let Some(vm_id) = crate::str_to_usize(arg) else {
            println!("\"{arg}\" is not a number");
            return true;
        };
        if crate::vm::switch_active_vm(vm_id) {
            println!("VM{vm_id} is actived");
            false
        } else {
            println!("VM{vm_id} is not available");
            true
        }
    }
}
