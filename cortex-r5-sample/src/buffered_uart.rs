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

// Board-specific imports (zynqmp takes precedence if both are enabled)
#[cfg(all(feature = "versatileab", not(feature = "zynqmp")))]
use crate::config::UART0_BASE;
#[cfg(feature = "zynqmp")]
use crate::config_zynqmp::{uart, UART0_BASE};
use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, Ordering};
use heapless::spsc::{Consumer, Producer, Queue};

/// Size of the transmit buffer (bytes)
const TX_BUFFER_SIZE: usize = 1024;

/// Wrapper to make UnsafeCell Sync for static usage.
///
/// # Safety
///
/// This impl is sound only under the SPSC discipline enforced by this module:
/// - `TX_PRODUCER` is accessed exclusively within critical sections (thread context)
/// - `TX_CONSUMER` is accessed exclusively from the UART ISR (single entry, no nesting)
/// - `TX_QUEUE` is split once during `init()` and never accessed directly after
///
/// The `heapless::spsc::Producer` and `Consumer` types are `!Sync` because they
/// assume single-owner access. Our critical section + ISR separation guarantees this.
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
static TX_QUEUE: SyncUnsafeCell<Queue<u8, TX_BUFFER_SIZE>> = SyncUnsafeCell::new(Queue::new());

/// Producer handle (protected by critical section)
static TX_PRODUCER: SyncUnsafeCell<Option<Producer<'static, u8, TX_BUFFER_SIZE>>> =
    SyncUnsafeCell::new(None);

/// Consumer handle (used only in ISR)
static TX_CONSUMER: SyncUnsafeCell<Option<Consumer<'static, u8, TX_BUFFER_SIZE>>> =
    SyncUnsafeCell::new(None);

/// Flag indicating if the buffered UART has been initialized
static INITIALIZED: AtomicBool = AtomicBool::new(false);

/// Counter for dropped bytes when queue is full
static DROPPED_BYTES: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

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
/// Disables TX interrupt during setup and LEAVES IT DISABLED.
/// The interrupt will be enabled on-demand when data is written.
pub unsafe fn init() {
    // Ensure init is only called once
    if INITIALIZED.swap(true, Ordering::AcqRel) {
        panic!("buffered_uart::init() called more than once");
    }

    // Disable TX interrupt during initialization
    disable_tx_interrupt();

    // Split the queue into producer and consumer
    let queue = &mut *TX_QUEUE.get();
    let (producer, consumer) = queue.split();
    *TX_PRODUCER.get() = Some(producer);
    *TX_CONSUMER.get() = Some(consumer);

    // Memory barrier to ensure producer/consumer are visible
    core::sync::atomic::fence(Ordering::Release);

    // LEAVE TX interrupt DISABLED - will be enabled on first write
}

/// Enable UART TX interrupt
///
/// # Safety
/// Must be called with interrupts disabled (inside critical section) to prevent
/// race condition during Read-Modify-Write of interrupt mask register.
#[inline(always)]
unsafe fn enable_tx_interrupt() {
    #[cfg(all(feature = "versatileab", not(feature = "zynqmp")))]
    {
        // PL011: IMSC register is read-modify-write
        let imsc_ptr = (UART0_BASE as *mut u32).add(crate::config::uart::IMSC_OFFSET);
        let current = imsc_ptr.read_volatile();
        imsc_ptr.write_volatile(current | crate::config::uart::IMSC_TXIM);
    }
    #[cfg(feature = "zynqmp")]
    {
        // Cadence UART: IER register enables interrupts (write-only, no RMW needed)
        let base = UART0_BASE as *mut u8;
        // TX FIFO empty interrupt (bit 3)
        const INTRPT_TXEMPTY: u32 = 1 << 3;
        base.add(uart::IER_OFFSET)
            .cast::<u32>()
            .write_volatile(INTRPT_TXEMPTY);
    }
}

/// Disable UART TX interrupt
///
/// # Safety
/// Safe to call from ISR context (interrupts already disabled at this priority).
/// When called from thread context, must be inside critical section.
#[inline(always)]
unsafe fn disable_tx_interrupt() {
    #[cfg(all(feature = "versatileab", not(feature = "zynqmp")))]
    {
        // PL011: IMSC register is read-modify-write
        let imsc_ptr = (UART0_BASE as *mut u32).add(crate::config::uart::IMSC_OFFSET);
        let current = imsc_ptr.read_volatile();
        imsc_ptr.write_volatile(current & !crate::config::uart::IMSC_TXIM);
    }
    #[cfg(feature = "zynqmp")]
    {
        // Cadence UART: IDR register disables interrupts (write-only)
        let base = UART0_BASE as *mut u8;
        const INTRPT_TXEMPTY: u32 = 1 << 3;
        base.add(uart::IDR_OFFSET)
            .cast::<u32>()
            .write_volatile(INTRPT_TXEMPTY);
    }
}

/// Thread-safe buffered UART writer
pub struct BufferedUartWriter;

impl ufmt::uWrite for BufferedUartWriter {
    type Error = core::convert::Infallible;

    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        // Check if initialized (early return if not)
        if !INITIALIZED.load(Ordering::Acquire) {
            return Ok(()); // Silently drop if not initialized
        }

        // Critical section: protect producer access AND IMSC register RMW
        // Both operations must be atomic to prevent race conditions:
        // 1. Producer enqueue must be serialized across threads
        // 2. IMSC enable must not race with ISR's disable
        critical_section::with(|_| unsafe {
            let producer_opt = &mut *TX_PRODUCER.get();
            if let Some(producer) = producer_opt {
                let mut enqueued = false;
                let mut dropped: u32 = 0;

                for &byte in s.as_bytes() {
                    if producer.enqueue(byte).is_ok() {
                        enqueued = true;
                    } else {
                        dropped += 1;
                    }
                }

                if dropped > 0 {
                    DROPPED_BYTES.fetch_add(dropped, Ordering::Relaxed);
                }

                if enqueued {
                    core::sync::atomic::fence(Ordering::Release);
                    enable_tx_interrupt();
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

    #[cfg(all(feature = "versatileab", not(feature = "zynqmp")))]
    {
        use crate::config::uart as pl011;
        let data_ptr = (UART0_BASE as *mut u32).add(pl011::DATA_OFFSET);
        let flag_ptr = (UART0_BASE as *const u32).add(pl011::FLAG_OFFSET);
        let icr_ptr = (UART0_BASE as *mut u32).add(pl011::ICR_OFFSET);

        // Clear the TX interrupt first to acknowledge hardware
        icr_ptr.write_volatile(pl011::ICR_TXIC);

        let consumer_opt = &mut *TX_CONSUMER.get();
        if let Some(consumer) = consumer_opt {
            // Send as many bytes as possible while FIFO has space
            while (flag_ptr.read_volatile() & pl011::FLAG_TXFF) == 0 {
                match consumer.dequeue() {
                    Some(byte) => {
                        data_ptr.write_volatile(byte as u32);
                    }
                    None => {
                        // Queue is empty, disable TX interrupt to avoid spurious interrupts
                        disable_tx_interrupt();
                        break;
                    }
                }
            }
        }
    }

    #[cfg(feature = "zynqmp")]
    {
        let base = UART0_BASE as *mut u8;
        let fifo_ptr = base.add(uart::FIFO_OFFSET).cast::<u32>();
        let status_ptr = base.add(uart::CHANNEL_STS_OFFSET).cast::<u32>();
        let isr_ptr = base.add(uart::ISR_OFFSET).cast::<u32>();

        // Clear only the TX-empty interrupt bit (W1C).
        // Writing back the full ISR snapshot would accidentally clear RX/error
        // interrupts that other handlers may need to process.
        const INTRPT_TXEMPTY: u32 = 1 << 3;
        let pending = isr_ptr.read_volatile();
        isr_ptr.write_volatile(pending & INTRPT_TXEMPTY);

        let consumer_opt = &mut *TX_CONSUMER.get();
        if let Some(consumer) = consumer_opt {
            // Send as many bytes as possible while FIFO has space
            while (status_ptr.read_volatile() & uart::STS_TXFULL) == 0 {
                match consumer.dequeue() {
                    Some(byte) => {
                        fifo_ptr.write_volatile(byte as u32);
                    }
                    None => {
                        // Queue is empty, disable TX interrupt to avoid spurious interrupts
                        disable_tx_interrupt();
                        break;
                    }
                }
            }
        }
    }

    // Data Synchronization Barrier: Ensure all MMIO writes complete before ISR exit
    // CRITICAL: Required on Cortex-R5 to guarantee interrupt clear is visible
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
///
/// Wraps the entire format operation in a single critical section so the
/// TX interrupt fires only once after all bytes are enqueued. Without this,
/// ufmt's multiple `write_str` calls (one per literal/argument/newline) each
/// enter and exit the critical section, allowing the TX ISR to fire between
/// fragments and interfere with ThreadX scheduling state.
#[macro_export]
macro_rules! println_uart {
    ($($arg:tt)*) => {{
        critical_section::with(|_| {
            let mut writer = $crate::buffered_uart::BufferedUartWriter;
            let _ = ufmt::uwriteln!(writer, $($arg)*);
        });
    }};
}

/// Print to buffered UART without newline
#[macro_export]
macro_rules! print_uart {
    ($($arg:tt)*) => {{
        critical_section::with(|_| {
            let mut writer = $crate::buffered_uart::BufferedUartWriter;
            let _ = ufmt::uwrite!(writer, $($arg)*);
        });
    }};
}
