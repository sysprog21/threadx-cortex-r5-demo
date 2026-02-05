//! Lock-free single-producer single-consumer (SPSC) ring buffer.
//!
//! Designed for ISR-to-thread messaging on single-core ARM systems.
//! The producer side (push) is lock-free and safe to call from interrupt context.
//! The consumer side (pop) is lock-free and safe to call from thread context.
//!
//! # ISR-to-Thread Pattern
//!
//! The canonical RTOS pattern for passing data from an interrupt to a thread:
//!
//! ```text
//! ISR (producer):
//!     spsc.push(data);
//!     tx_event_flags_set(&EVENTS, FLAG_RX, TX_OR);
//!
//! Thread (consumer):
//!     tx_event_flags_get(&EVENTS, FLAG_RX, TX_AND_CLEAR, ...);
//!     while let Some(val) = spsc.pop() { process(val); }
//! ```
//!
//! This avoids calling `tx_queue_send` from ISR context (which may block or
//! require TX_NO_WAIT error handling) and provides predictable O(1) push/pop.
//!
//! # Why Not `tx_queue`?
//!
//! ThreadX queues use internal locking and can suspend the caller. In ISR context,
//! suspension is forbidden. A lock-free SPSC ring paired with event flags gives:
//! - Zero-copy element transfer (no intermediate buffer)
//! - No critical sections on the hot path
//! - Bounded, predictable latency
//! - Clean separation: ring handles data, event flags handle notification
//!
//! # Memory Ordering
//!
//! Uses Acquire/Release ordering on head and tail indices:
//! - Producer: writes data, then stores head with Release (data visible before index)
//! - Consumer: reads data after loading head with Acquire (sees producer's writes)
//! - Symmetric for tail updates
//!
//! Correct for single-core ARM Cortex-R/M where ISR and thread share coherent memory.

use core::cell::UnsafeCell;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicUsize, Ordering};

/// Lock-free SPSC ring buffer with compile-time capacity.
///
/// `T` must be `Copy` to ensure safe byte-level transfer without drop concerns.
/// `N` must be a power of two - this is enforced at compile time. Power-of-two
/// capacity enables bitmask indexing (`& (N-1)` instead of `% N`) and guarantees
/// correct wrapping arithmetic when indices overflow `usize::MAX`.
///
/// # Static Usage
///
/// ```rust,ignore
/// static RING: Spsc<u32, 16> = Spsc::new();
///
/// // In ISR (producer):
/// unsafe { RING.push(sensor_reading).ok(); }
///
/// // In thread (consumer):
/// while let Some(val) = unsafe { RING.pop() } { /* ... */ }
/// ```
pub struct Spsc<T: Copy, const N: usize> {
    /// Write index, monotonically increasing. Only modified by producer.
    /// Aligned to 64 bytes to prevent false sharing with `tail`.
    head: CacheAligned<AtomicUsize>,
    /// Read index, monotonically increasing. Only modified by consumer.
    tail: AtomicUsize,
    /// Ring buffer slots.
    buf: [UnsafeCell<MaybeUninit<T>>; N],
}

/// Cache-line-aligned wrapper to prevent false sharing.
///
/// ARM Cortex-R5 has 32-byte cache lines; 64 bytes covers common sizes
/// without relying on compile-time arithmetic that could break portability.
#[repr(align(64))]
struct CacheAligned<T>(T);

// SAFETY: Spsc is designed for exactly one producer and one consumer accessing
// it concurrently. Violating this contract (e.g., two threads calling push())
// is undefined behavior - there is no internal locking to prevent data races
// on buffer slots. AtomicUsize indices provide Acquire/Release synchronization
// between the single producer and single consumer. UnsafeCell allows interior
// mutability of buffer slots, which is safe under SPSC discipline because each
// slot is exclusively written by the producer and exclusively read by the
// consumer, with proper ordering between the data access and the index update.
unsafe impl<T: Copy + Send, const N: usize> Sync for Spsc<T, N> {}
unsafe impl<T: Copy + Send, const N: usize> Send for Spsc<T, N> {}

impl<T: Copy, const N: usize> Spsc<T, N> {
    /// Creates a new empty SPSC ring buffer.
    ///
    /// Can be used in static initializers:
    /// ```rust,ignore
    /// static RING: Spsc<u32, 8> = Spsc::new();
    /// ```
    ///
    /// # Compile-Time Checks
    ///
    /// Panics at compile time if `N` is zero or not a power of two.
    pub const fn new() -> Self {
        assert!(N > 0, "SPSC capacity must be non-zero");
        assert!(N & (N - 1) == 0, "SPSC capacity must be a power of two");
        Self {
            head: CacheAligned(AtomicUsize::new(0)),
            tail: AtomicUsize::new(0),
            // SAFETY: MaybeUninit<T> has no initialization requirement.
            // UnsafeCell is repr(transparent), adding no layout constraints.
            // An uninitialized array of UnsafeCell<MaybeUninit<T>> is valid.
            buf: unsafe { MaybeUninit::<[UnsafeCell<MaybeUninit<T>>; N]>::uninit().assume_init() },
        }
    }

    /// Returns the capacity of the ring buffer.
    #[inline]
    pub const fn capacity(&self) -> usize {
        N
    }

    /// Returns the number of elements currently in the ring.
    ///
    /// This is a non-linearizable snapshot: the returned value may be stale
    /// or transiently exceed `N` if called concurrently with push/pop.
    #[inline]
    pub fn len(&self) -> usize {
        let head = self.head.0.load(Ordering::Acquire);
        let tail = self.tail.load(Ordering::Acquire);
        head.wrapping_sub(tail)
    }

    /// Returns true if the ring is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Pushes a value into the ring (producer side).
    ///
    /// Returns `Err(val)` if the ring is full, returning ownership of the value.
    /// This operation is O(1) and lock-free.
    ///
    /// # Safety
    ///
    /// Must only be called from a single producer context. Concurrent calls
    /// from multiple producers cause data races. Safe to call from ISR context
    /// on single-core systems.
    #[inline]
    pub unsafe fn push(&self, val: T) -> Result<(), T> {
        let head = self.head.0.load(Ordering::Relaxed);
        let tail = self.tail.load(Ordering::Acquire);

        if head.wrapping_sub(tail) >= N {
            return Err(val);
        }

        self.buf[head & (N - 1)].get().write(MaybeUninit::new(val));

        // Release: data write must be visible before head update
        self.head.0.store(head.wrapping_add(1), Ordering::Release);

        Ok(())
    }

    /// Pops a value from the ring (consumer side).
    ///
    /// Returns `None` if the ring is empty.
    /// This operation is O(1) and lock-free.
    ///
    /// # Safety
    ///
    /// Must only be called from a single consumer context. Concurrent calls
    /// from multiple consumers cause data races.
    #[inline]
    pub unsafe fn pop(&self) -> Option<T> {
        let tail = self.tail.load(Ordering::Relaxed);
        let head = self.head.0.load(Ordering::Acquire);

        if head == tail {
            return None;
        }

        // SAFETY: Slot was written by producer before head was updated (Release).
        // We loaded head with Acquire, so the data write is visible.
        let val = self.buf[tail & (N - 1)].get().read().assume_init();

        // Release: read must complete before tail update (frees slot for producer)
        self.tail.store(tail.wrapping_add(1), Ordering::Release);

        Some(val)
    }

    /// Splits the ring into producer and consumer halves.
    ///
    /// Provides compile-time enforcement of the single-producer single-consumer
    /// contract via the borrow checker. Each half requires `&mut self` for
    /// operations, preventing aliased access.
    ///
    /// For static usage where split is impractical (ISR + thread with separate
    /// static references), use the unsafe [`push`](Spsc::push)/[`pop`](Spsc::pop)
    /// methods directly.
    pub fn split(&mut self) -> (Producer<'_, T, N>, Consumer<'_, T, N>) {
        (Producer { rb: self }, Consumer { rb: self })
    }
}

/// Producer half of a SPSC ring buffer.
///
/// Obtained by calling [`Spsc::split`]. The `&mut self` requirement on
/// [`push`](Producer::push) prevents multiple simultaneous producers.
pub struct Producer<'a, T: Copy, const N: usize> {
    rb: &'a Spsc<T, N>,
}

impl<T: Copy, const N: usize> Producer<'_, T, N> {
    /// Pushes a value into the ring.
    ///
    /// Returns `Err(val)` if the ring is full.
    #[inline]
    pub fn push(&mut self, val: T) -> Result<(), T> {
        // SAFETY: Split guarantees exclusive producer access.
        // &mut self prevents aliased push calls.
        unsafe { self.rb.push(val) }
    }

    /// Returns true if the ring is full.
    #[inline]
    pub fn is_full(&self) -> bool {
        let head = self.rb.head.0.load(Ordering::Relaxed);
        let tail = self.rb.tail.load(Ordering::Acquire);
        head.wrapping_sub(tail) >= N
    }

    /// Returns the number of elements in the ring.
    #[inline]
    pub fn len(&self) -> usize {
        self.rb.len()
    }

    /// Returns true if the ring is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.rb.is_empty()
    }
}

/// Consumer half of a SPSC ring buffer.
///
/// Obtained by calling [`Spsc::split`]. The `&mut self` requirement on
/// [`pop`](Consumer::pop) prevents multiple simultaneous consumers.
pub struct Consumer<'a, T: Copy, const N: usize> {
    rb: &'a Spsc<T, N>,
}

impl<T: Copy, const N: usize> Consumer<'_, T, N> {
    /// Pops a value from the ring.
    ///
    /// Returns `None` if the ring is empty.
    #[inline]
    pub fn pop(&mut self) -> Option<T> {
        // SAFETY: Split guarantees exclusive consumer access.
        // &mut self prevents aliased pop calls.
        unsafe { self.rb.pop() }
    }

    /// Returns true if the ring is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        let tail = self.rb.tail.load(Ordering::Relaxed);
        let head = self.rb.head.0.load(Ordering::Acquire);
        head == tail
    }

    /// Returns the number of elements in the ring.
    #[inline]
    pub fn len(&self) -> usize {
        self.rb.len()
    }
}
