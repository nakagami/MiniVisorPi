//!
//! Mutex<T> implementation using a fair (ticket-based) Spin Lock
//!

use crate::asm::{get_daif_and_disable_irq_fiq, set_daif};

use core::cell::UnsafeCell;
use core::hint::spin_loop;
use core::marker::PhantomData;
use core::ops::{Deref, DerefMut};
use core::sync::atomic::{AtomicU32, Ordering};

/// A ticket lock rather than a plain test-and-set spin lock: on this hypervisor, several
/// pCPUs concurrently running vCPUs of the same true-SMP guest (see `vm::activate_vm_on_this_pcpu`)
/// can all repeatedly contend for the same global lock at very high frequency and for a long
/// time -- e.g. every idle vCPU's guest WFI trap (`exception::wfx_handler`) unconditionally
/// polls the physical UART (`PL011_DEVICE`) on every single trap, so with several idle pCPUs
/// looping this nonstop, a plain `compare_exchange`-based lock has no fairness guarantee and
/// lets a subset of cores keep winning the race indefinitely, starving another core out of the
/// lock forever. That starvation is externally indistinguishable from a true deadlock: the
/// starved core spins with interrupts disabled (see `get_daif_and_disable_irq_fiq`) for as long
/// as it keeps losing, and if that core happens to be the one whose own local Generic Timer PPI
/// would otherwise fire, its vCPU's scheduler tick permanently stops arriving (an RCU stall /
/// "hangs before login" symptom despite every physical device and interrupt being wired up
/// correctly). A ticket lock guarantees strict FIFO ordering among waiters, giving every
/// contending pCPU a bounded wait and eliminating this class of starvation entirely.
pub struct Mutex<T: ?Sized> {
    next_ticket: AtomicU32,
    now_serving: AtomicU32,
    data: UnsafeCell<T>,
}

pub struct MutexGuard<'a, T: ?Sized + 'a> {
    now_serving: &'a AtomicU32,
    daif: u64,
    data: &'a mut T,
    _forbid_send: PhantomData<*const ()>,
}

impl<T> Mutex<T> {
    pub const fn new(d: T) -> Mutex<T> {
        Mutex {
            next_ticket: AtomicU32::new(0),
            now_serving: AtomicU32::new(0),
            data: UnsafeCell::new(d),
        }
    }
}

impl<T: ?Sized> Mutex<T> {
    pub fn lock(&self) -> MutexGuard<'_, T> {
        /* Draw this waiter's own ticket (its position in the FIFO queue) and keep it: once
         * drawn, it must never be re-drawn (unlike a `compare_exchange` retry loop), or the
         * strict ordering this whole scheme exists to provide would be lost. */
        let my_ticket = self.next_ticket.fetch_add(1, Ordering::Relaxed);
        /* Busy-wait for our turn with interrupts left alone, so a physical interrupt this
         * pCPU is itself responsible for (e.g. its own Generic Timer PPI) can still be taken
         * while merely waiting rather than actually holding the lock. */
        while self.now_serving.load(Ordering::Acquire) != my_ticket {
            spin_loop();
        }
        /* It is now (and, since `now_serving` only ever advances forward and no other
         * waiter can ever match `my_ticket`, will remain) our turn: only mask interrupts
         * for the critical section itself. */
        let daif = unsafe { get_daif_and_disable_irq_fiq() };
        MutexGuard {
            now_serving: &self.now_serving,
            daif,
            data: unsafe { &mut *self.data.get() },
            _forbid_send: PhantomData,
        }
    }
}

unsafe impl<T: ?Sized + Send> Send for Mutex<T> {}
unsafe impl<T: ?Sized + Send> Sync for Mutex<T> {}

//impl<T: ?Sized> !Send for MutexGuard<'_, T> {}
unsafe impl<T: ?Sized + Sync> Sync for MutexGuard<'_, T> {}

impl<T: ?Sized> Deref for MutexGuard<'_, T> {
    type Target = T;
    fn deref(&self) -> &T {
        &*self.data
    }
}

impl<T: ?Sized> DerefMut for MutexGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        &mut *self.data
    }
}

impl<T: ?Sized> Drop for MutexGuard<'_, T> {
    fn drop(&mut self) {
        /* Hand the lock to the next ticket in line (wrapping is fine: `now_serving` and
         * `next_ticket` wrap together modulo 2^32, and no more than a handful of pCPUs are
         * ever actually contending at once). */
        self.now_serving.fetch_add(1, Ordering::Release);
        unsafe { set_daif(self.daif) };
    }
}
