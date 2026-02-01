//! Interrupt-driven buffered UART with lock-free queue.
//!
//! This provides high-performance logging with minimal interrupt latency:
//! - Producer: Pushes formatted strings to lock-free queue (fast, <10µs critical section)
//! - Consumer: UART TX interrupt drains queue asynchronously
//! - No busy-waiting with interrupts disabled
//! - Safe for use from multiple threads on **single-core systems only**
//!
//! # Safety Requirements
//!
//! - **Single-Core Only**: This implementation is NOT safe for SMP (symmetric multiprocessing).
//!   The critical_section implementation must provide global mutual exclusion.
//! - **Exclusive UART Ownership**: No other code may manipulate the UART interrupt mask.
//! - **Initialization**: Must call `init()` exactly once before any logging operations.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, Ordering};
use heapless::spsc::{Consumer, Producer, Queue};

/// Size of the transmit buffer (bytes)
const TX_BUFFER_SIZE: usize = 1024;

/// Wrapper to make UnsafeCell Sync for static usage
struct SyncUnsafeCell<T>(UnsafeCell<T>);
unsafe impl<T> Sync for SyncUnsafeCell<T> {}

impl<T> SyncUnsafeCell<T> {
    const fn new(value: T) -> Self {
        Self(UnsafeCell::new(value))
    }

    fn get(&self) -> *mut T {
        self.0.get()
    }
}

/// Global TX queue for buffered UART output
static TX_QUEUE: SyncUnsafeCell<Queue<u8, TX_BUFFER_SIZE>> =
    SyncUnsafeCell::new(Queue::new());

/// Producer handle (protected by critical section)
static TX_PRODUCER: SyncUnsafeCell<Option<Producer<'static, u8, TX_BUFFER_SIZE>>> =
    SyncUnsafeCell::new(None);

/// Consumer handle (used only in ISR)
static TX_CONSUMER: SyncUnsafeCell<Option<Consumer<'static, u8, TX_BUFFER_SIZE>>> =
    SyncUnsafeCell::new(None);

/// Flag indicating if TX interrupt is enabled
static TX_INTERRUPT_ENABLED: AtomicBool = AtomicBool::new(false);

/// Flag indicating if the buffered UART has been initialized
static INITIALIZED: AtomicBool = AtomicBool::new(false);

/// Counter for dropped bytes when queue is full
static DROPPED_BYTES: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// PL011 UART register offsets
const UART_BASE: usize = 0x101f_1000;
const UART_DATA_OFFSET: usize = 0x000 >> 2;
const UART_FLAG_OFFSET: usize = 0x018 >> 2;
const UART_IMSC_OFFSET: usize = 0x038 >> 2;  // Interrupt Mask Set/Clear
const UART_ICR_OFFSET: usize = 0x044 >> 2;   // Interrupt Clear

const UART_FLAG_TXFF: u32 = 1 << 5;  // TX FIFO Full
const UART_IMSC_TXIM: u32 = 1 << 5;  // TX Interrupt Mask
const UART_ICR_TXIC: u32 = 1 << 5;   // Clear TX Interrupt

/// Initialize the buffered UART system
///
/// # Safety
/// Must be called once during system initialization before any threads start.
/// Must be called after UART hardware is initialized.
///
/// # Panics
/// Panics if called more than once.
///
/// # Implementation Notes
/// Disables TX interrupt before setup to prevent ISR running with uninitialized state,
/// then enables it at the end.
pub unsafe fn init() {
    // Ensure init is only called once
    if INITIALIZED.swap(true, Ordering::AcqRel) {
        panic!("buffered_uart::init() called more than once");
    }

    // Disable TX interrupt during initialization to prevent ISR with uninitialized state
    disable_tx_interrupt();

    // Split the queue into producer and consumer
    let queue = &mut *TX_QUEUE.get();
    let (producer, consumer) = queue.split();
    *TX_PRODUCER.get() = Some(producer);
    *TX_CONSUMER.get() = Some(consumer);

    // Memory barrier to ensure producer/consumer are visible before enabling interrupt
    core::sync::atomic::fence(Ordering::Release);

    // Enable UART TX interrupt now that setup is complete
    enable_tx_interrupt();
    TX_INTERRUPT_ENABLED.store(true, Ordering::Release);
}

/// Enable UART TX interrupt
unsafe fn enable_tx_interrupt() {
    let imsc_ptr = (UART_BASE as *mut u32).add(UART_IMSC_OFFSET);
    let current = imsc_ptr.read_volatile();
    imsc_ptr.write_volatile(current | UART_IMSC_TXIM);
}

/// Disable UART TX interrupt
unsafe fn disable_tx_interrupt() {
    let imsc_ptr = (UART_BASE as *mut u32).add(UART_IMSC_OFFSET);
    let current = imsc_ptr.read_volatile();
    imsc_ptr.write_volatile(current & !UART_IMSC_TXIM);
}

/// Thread-safe buffered UART writer
pub struct BufferedUartWriter;

impl core::fmt::Write for BufferedUartWriter {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        // Check if initialized (early return if not)
        if !INITIALIZED.load(Ordering::Acquire) {
            return Ok(()); // Silently drop if not initialized
        }

        // Short critical section: only to access the producer
        critical_section::with(|_| {
            unsafe {
                let producer_opt = &mut *TX_PRODUCER.get();
                if let Some(producer) = producer_opt {
                    // Try to enqueue all bytes
                    let mut dropped = 0;
                    for &byte in s.as_bytes() {
                        // If queue is full, drop the character (non-blocking)
                        if producer.enqueue(byte).is_err() {
                            dropped += 1;
                        }
                    }

                    // Track dropped bytes for diagnostics
                    if dropped > 0 {
                        DROPPED_BYTES.fetch_add(dropped, Ordering::Relaxed);
                    }

                    // Memory fence: Ensure enqueue operations are visible before enabling interrupt
                    // This prevents lost-wakeup race where ISR sees empty queue after we enable it
                    core::sync::atomic::fence(Ordering::Release);

                    // Always enable TX interrupt to ensure queue gets drained
                    // This is idempotent and prevents state drift issues
                    enable_tx_interrupt();
                    TX_INTERRUPT_ENABLED.store(true, Ordering::Release);
                }
            }
        });
        Ok(())
    }
}

/// UART TX interrupt handler
///
/// Called by the main interrupt dispatcher when UART TX interrupt fires.
/// Drains the queue and sends characters to UART hardware.
///
/// # Safety
/// Must only be called from the UART interrupt context.
pub unsafe fn handle_tx_interrupt() {
    // Check if initialized (early return if not)
    if !INITIALIZED.load(Ordering::Acquire) {
        return; // Silently ignore if not initialized
    }

    let data_ptr = (UART_BASE as *mut u32).add(UART_DATA_OFFSET);
    let flag_ptr = (UART_BASE as *const u32).add(UART_FLAG_OFFSET);
    let icr_ptr = (UART_BASE as *mut u32).add(UART_ICR_OFFSET);

    // Clear the TX interrupt first to acknowledge hardware
    icr_ptr.write_volatile(UART_ICR_TXIC);

    let consumer_opt = &mut *TX_CONSUMER.get();
    if let Some(consumer) = consumer_opt {
        // Send as many bytes as possible while FIFO has space
        while (flag_ptr.read_volatile() & UART_FLAG_TXFF) == 0 {
            match consumer.dequeue() {
                Some(byte) => {
                    data_ptr.write_volatile(byte as u32);
                }
                None => {
                    // Queue is empty, disable TX interrupt to avoid spurious interrupts
                    disable_tx_interrupt();
                    TX_INTERRUPT_ENABLED.store(false, Ordering::Release);
                    break;
                }
            }
        }
    }

    // Data Synchronization Barrier: Ensure all MMIO writes complete before ISR exit
    // CRITICAL: Required for PL011 on Cortex-R5 to guarantee interrupt clear is visible
    // to hardware before IRQ controller deasserts. Without this, spurious re-entry is possible.
    core::arch::asm!("dsb sy", options(nostack, preserves_flags));
}

/// Get the number of dropped bytes due to buffer overflow
///
/// This can be used for diagnostics to detect when the log buffer is too small
/// or the application is logging too aggressively.
pub fn get_dropped_bytes() -> u32 {
    DROPPED_BYTES.load(Ordering::Relaxed)
}

/// Print to buffered UART with newline
#[macro_export]
macro_rules! println_uart {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        let mut writer = $crate::buffered_uart::BufferedUartWriter;
        let _ = writeln!(writer, $($arg)*);
    }};
}

/// Print to buffered UART without newline
#[macro_export]
macro_rules! print_uart {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        let mut writer = $crate::buffered_uart::BufferedUartWriter;
        let _ = write!(writer, $($arg)*);
    }};
}
