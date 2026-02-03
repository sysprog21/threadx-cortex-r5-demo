//! Rust demonstration for a QEMU Arm Cortex-R machine running ThreadX.
//!
//! This binary is not compiled when the `host-test` feature is enabled.

// Guard entire binary for embedded target only
#![cfg(not(feature = "host-test"))]
#![no_std]
#![no_main]

use core::fmt::Write as _;
use cortex_r5_sample::{
    config::{TIMER0_IRQ, UART0_BASE, UART0_IRQ, VIC_BASE},
    pl011_uart::Uart,
    pl190_vic,
    sp804_timer::{self, Timer0},
};
use static_cell::StaticCell;

// Import safe UART logging macros
use cortex_r5_sample::println_uart;

/// ThreadX critical section implementation for single-core systems.
///
/// # Safety
/// This implementation disables local interrupts via ThreadX interrupt control.
/// It provides mutual exclusion ONLY on single-core systems. The save/restore
/// pattern properly handles nested critical sections.
///
/// ## Limitations
/// - **Single-Core Only**: Do not use with ThreadX SMP - interrupt disabling
///   only affects the local core and does not prevent other cores from accessing
///   protected data.
/// - **No Blocking APIs**: Never call ThreadX blocking APIs (mutex, semaphore,
///   queue with wait) inside a critical section - this will deadlock the system.
/// - **Short Duration Only**: Keep critical sections extremely short (< 10 μs)
///   to avoid interrupt latency spikes and scheduler starvation.
/// - **ISR Compatible**: Safe to call from both thread and interrupt context.
///
/// ## Integration Warning
/// The existing `GlobalUart` uses a ThreadX mutex. Do NOT call UART logging
/// functions from inside critical sections - use a non-blocking ring buffer
/// pattern for defmt/logging integration.
struct ThreadXCriticalSection;
critical_section::set_impl!(ThreadXCriticalSection);

unsafe impl critical_section::Impl for ThreadXCriticalSection {
    unsafe fn acquire() -> critical_section::RawRestoreState {
        // Disable interrupts and get previous state
        let state = threadx_sys::_tx_thread_interrupt_control(threadx_sys::TX_INT_DISABLE);

        // Compiler fence prevents reordering of memory operations across the
        // critical section boundary. Acquire semantics ensure all loads/stores
        // after this fence observe the effects of the critical section entry.
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Acquire);

        state
    }
    unsafe fn release(restore_state: critical_section::RawRestoreState) {
        // Compiler fence prevents reordering of memory operations across the
        // critical section boundary. Release semantics ensure all loads/stores
        // within the critical section complete before the fence.
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);

        // Restore previous interrupt state (preserves nesting depth)
        threadx_sys::_tx_thread_interrupt_control(restore_state);
    }
}

const DEMO_STACK_SIZE: usize = 16384;
const DEMO_POOL_SIZE: usize = (DEMO_STACK_SIZE * 2) + 16384;

// Scenario B: Producer-consumer pipeline depth (number of messages)
const PIPELINE_DEPTH: u32 = 5;

// Scenario C: Mutex contention rounds (iterations under lock)
const MUTEX_ROUNDS: u32 = 10;

// Global primitives for runtime validation (initialized in tx_application_define)
static TEST_MUTEX: StaticCell<threadx_sys::TX_MUTEX> = StaticCell::new();
static TEST_SEMA: StaticCell<threadx_sys::TX_SEMAPHORE> = StaticCell::new();
static TEST_FLAGS: StaticCell<threadx_sys::TX_EVENT_FLAGS_GROUP> = StaticCell::new();
static TEST_QUEUE: StaticCell<threadx_sys::TX_QUEUE> = StaticCell::new();
static TEST_QUEUE_STORAGE: StaticCell<[u32; 4]> = StaticCell::new();

// Atomic pointers to primitives for thread access (set after initialization).
// Uses Release (store) / Acquire (load) ordering to ensure visibility across threads.
static QUEUE_PTR: core::sync::atomic::AtomicPtr<threadx_sys::TX_QUEUE> =
    core::sync::atomic::AtomicPtr::new(core::ptr::null_mut());
static MUTEX_PTR: core::sync::atomic::AtomicPtr<threadx_sys::TX_MUTEX> =
    core::sync::atomic::AtomicPtr::new(core::ptr::null_mut());
static FLAGS_PTR: core::sync::atomic::AtomicPtr<threadx_sys::TX_EVENT_FLAGS_GROUP> =
    core::sync::atomic::AtomicPtr::new(core::ptr::null_mut());

// Shared counter for mutex contention test (Scenario C).
// Accessed under mutex protection - Relaxed ordering is sufficient because
// the mutex provides the happens-before relationship.
static SHARED_COUNTER: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Stack error handler invoked when ThreadX detects stack corruption.
///
/// This is called when stack checking discovers that a thread has overflowed
/// its stack. ThreadX fills unused stack space with 0xEFEF pattern and validates
/// this pattern on context switches when TX_ENABLE_STACK_CHECKING is enabled.
///
/// # Safety
/// This function is called from ThreadX kernel context (during context switch)
/// where blocking operations are forbidden. It uses emergency UART bypass to
/// avoid mutex deadlocks and minimizes stack usage to prevent further corruption.
///
/// # Implementation Notes
/// - Uses `emergency_write_str()` to bypass ThreadX mutex on UART
/// - Minimal stack usage (no format machinery beyond simple writes)
/// - Validates pointers defensively - corruption may have trashed TCB
/// - Halts CPU with interrupts disabled instead of panic (no unwinding)
#[no_mangle]
extern "C" fn stack_error_handler(thread_ptr: *mut threadx_sys::TX_THREAD_STRUCT) {
    use core::fmt::Write;

    // Emergency writer that bypasses all mutexes
    struct EmergencyWriter;
    impl core::fmt::Write for EmergencyWriter {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            unsafe {
                Uart::<UART0_BASE>::emergency_write_str(s);
            }
            Ok(())
        }
    }

    let mut w = EmergencyWriter;

    let _ = writeln!(w, "\r\n!!! STACK OVERFLOW DETECTED !!!");

    // Validate pointer before dereferencing
    if thread_ptr.is_null() {
        let _ = writeln!(w, "Thread: NULL pointer");
    } else {
        // Print pointer first in case struct is corrupted
        let _ = writeln!(w, "Thread @ {:p}", thread_ptr);

        // SAFETY: We're in a stack corruption scenario - the TCB may be partially
        // corrupted. Use volatile reads to prevent compiler optimizations that
        // might cause issues, and validate each field before use.
        unsafe {
            // Check if pointer is aligned (basic sanity check)
            if (thread_ptr as usize) % core::mem::align_of::<threadx_sys::TX_THREAD_STRUCT>() != 0 {
                let _ = writeln!(w, "Thread pointer misaligned - TCB corrupted");
            } else {
                // Read fields using volatile to prevent optimization issues
                let name_ptr = core::ptr::read_volatile(&(*thread_ptr).tx_thread_name);

                // Attempt to read name carefully (may be corrupted)
                if !name_ptr.is_null() {
                    // Check if name pointer looks valid (within reasonable range)
                    let name_addr = name_ptr as usize;
                    // Sanity check: name should be in a reasonable memory range
                    if name_addr > 0x1000 && name_addr < 0x8000_0000 {
                        let _ = write!(w, "Name: ");
                        // Read one byte at a time to minimize risk
                        for i in 0..32usize {
                            let byte_ptr = (name_ptr as *const u8).add(i);
                            let b = core::ptr::read_volatile(byte_ptr);
                            if b == 0 {
                                break;
                            }
                            if b.is_ascii_graphic() || b == b' ' {
                                let _ = write!(w, "{}", b as char);
                            } else {
                                let _ = write!(w, "?");
                            }
                        }
                        let _ = writeln!(w);
                    } else {
                        let _ = writeln!(w, "Name: <invalid pointer {:p}>", name_ptr);
                    }
                }

                // Read stack info using volatile
                let stack_start = core::ptr::read_volatile(&(*thread_ptr).tx_thread_stack_start);
                let stack_end = core::ptr::read_volatile(&(*thread_ptr).tx_thread_stack_end);
                let stack_size = core::ptr::read_volatile(&(*thread_ptr).tx_thread_stack_size);
                let stack_ptr = core::ptr::read_volatile(&(*thread_ptr).tx_thread_stack_ptr);

                let _ = writeln!(
                    w,
                    "Stack: {:p} - {:p} (size={})",
                    stack_start, stack_end, stack_size
                );
                let _ = writeln!(w, "Stack SP: {:p}", stack_ptr);
            }
        }
    }

    let _ = writeln!(w, "System halted.");

    // Disable interrupts and halt CPU
    // Do NOT use panic!() - it may try to acquire locks
    unsafe {
        core::arch::asm!(
            "cpsid i", // Disable interrupts (CPSR I bit)
            "2:",
            "wfi",  // Wait for interrupt (low power)
            "b 2b", // Loop forever
            options(noreturn)
        );
    }
}

/// Initializes the application.
///
/// This is invoked by ThreadX during scheduler start-up and is used to create
/// threads.
#[no_mangle]
extern "C" fn tx_application_define(_first_unused_memory: *mut core::ffi::c_void) {
    println_uart!("In tx_application_define()...");

    // Register stack error notification handler
    unsafe {
        threadx_sys::_tx_thread_stack_error_notify(Some(stack_error_handler));
    }

    // ThreadX requires a mutable pointer to a character array for object names,
    // which it retains internally. Use mutable static buffers to avoid casting
    // read-only string literals to *mut CHAR (which would be UB if ThreadX writes).
    // Use addr_of_mut! to get raw pointers without creating mutable references.
    static mut BYTE_POOL_NAME: [u8; 16] = *b"byte-pool0\0\0\0\0\0\0";
    static mut THREAD0_NAME: [u8; 16] = *b"producer\0\0\0\0\0\0\0\0";
    static mut THREAD1_NAME: [u8; 16] = *b"consumer\0\0\0\0\0\0\0\0";
    static mut MUTEX_NAME: [u8; 16] = *b"test-mutex\0\0\0\0\0\0";
    static mut SEMA_NAME: [u8; 16] = *b"test-sema\0\0\0\0\0\0\0";
    static mut FLAGS_NAME: [u8; 16] = *b"test-flags\0\0\0\0\0\0";
    static mut QUEUE_NAME: [u8; 16] = *b"test-queue\0\0\0\0\0\0";

    let byte_pool = {
        static BYTE_POOL: StaticCell<threadx_sys::TX_BYTE_POOL> = StaticCell::new();
        static BYTE_POOL_STORAGE: StaticCell<[u8; DEMO_POOL_SIZE]> = StaticCell::new();
        let byte_pool = BYTE_POOL.uninit();
        let byte_pool_storage = BYTE_POOL_STORAGE.uninit();
        unsafe {
            let res = threadx_sys::_tx_byte_pool_create(
                byte_pool.as_mut_ptr(),
                core::ptr::addr_of_mut!(BYTE_POOL_NAME) as *mut threadx_sys::CHAR,
                byte_pool_storage.as_mut_ptr() as *mut _,
                DEMO_POOL_SIZE as u32,
            );
            if res != threadx_sys::TX_SUCCESS {
                panic!("Failed to create byte pool: {}", res);
            }
            byte_pool.assume_init_mut()
        }
    };

    // ============================================================
    // Scenario A: Primitive Creation
    // Creates all synchronization primitives used by later scenarios.
    // No synthetic operations - real validation happens in thread context.
    // ============================================================

    // Mutex with TX_INHERIT for priority inheritance (needed by Scenario C)
    unsafe {
        let mutex = TEST_MUTEX.uninit();
        let res = threadx_sys::_tx_mutex_create(
            mutex.as_mut_ptr(),
            core::ptr::addr_of_mut!(MUTEX_NAME) as *mut threadx_sys::CHAR,
            threadx_sys::TX_INHERIT,
        );
        if res != threadx_sys::TX_SUCCESS {
            panic!("Failed to create mutex: {}", res);
        }
        let mutex = mutex.assume_init_mut();
        MUTEX_PTR.store(mutex, core::sync::atomic::Ordering::Release);
    }

    // Semaphore (initial count 0)
    unsafe {
        let sema = TEST_SEMA.uninit();
        let res = threadx_sys::_tx_semaphore_create(
            sema.as_mut_ptr(),
            core::ptr::addr_of_mut!(SEMA_NAME) as *mut threadx_sys::CHAR,
            0,
        );
        if res != threadx_sys::TX_SUCCESS {
            panic!("Failed to create semaphore: {}", res);
        }
    }

    // Event flags (coordination between threads)
    unsafe {
        let flags = TEST_FLAGS.uninit();
        let res = threadx_sys::_tx_event_flags_create(
            flags.as_mut_ptr(),
            core::ptr::addr_of_mut!(FLAGS_NAME) as *mut threadx_sys::CHAR,
        );
        if res != threadx_sys::TX_SUCCESS {
            panic!("Failed to create event flags: {}", res);
        }
        let flags = flags.assume_init_mut();
        FLAGS_PTR.store(flags, core::sync::atomic::Ordering::Release);
    }

    // Queue (small buffer for producer-consumer pipeline)
    unsafe {
        let queue = TEST_QUEUE.uninit();
        let queue_storage = TEST_QUEUE_STORAGE.uninit();
        let res = threadx_sys::_tx_queue_create(
            queue.as_mut_ptr(),
            core::ptr::addr_of_mut!(QUEUE_NAME) as *mut threadx_sys::CHAR,
            1, // Message size: 1 u32 word
            queue_storage.as_mut_ptr() as *mut core::ffi::c_void,
            (4 * 4) as u32, // 4 slots * 4 bytes = 16 bytes
        );
        if res != threadx_sys::TX_SUCCESS {
            panic!("Failed to create queue: {}", res);
        }
        let queue = queue.assume_init_mut();
        QUEUE_PTR.store(queue, core::sync::atomic::Ordering::Release);
    }

    println_uart!("PRIMITIVES_OK");

    // ============================================================
    // Thread Creation
    // ============================================================

    // Thread roles passed as entry parameter
    const ROLE_PRODUCER: u32 = 0;
    const ROLE_CONSUMER: u32 = 1;

    let thread0 = {
        let mut stack_pointer = core::ptr::null_mut();
        unsafe {
            let res = threadx_sys::_tx_byte_allocate(
                byte_pool,
                &mut stack_pointer,
                DEMO_STACK_SIZE as _,
                threadx_sys::TX_NO_WAIT,
            );
            if res != threadx_sys::TX_SUCCESS {
                panic!("Failed to allocate stack: {}", res);
            }
        }
        if stack_pointer.is_null() {
            panic!("Stack allocation returned null pointer");
        }

        static THREAD_STORAGE: StaticCell<threadx_sys::TX_THREAD> = StaticCell::new();
        let thread = THREAD_STORAGE.uninit();
        unsafe {
            let res = threadx_sys::_tx_thread_create(
                thread.as_mut_ptr(),
                core::ptr::addr_of_mut!(THREAD0_NAME) as *mut threadx_sys::CHAR,
                Some(my_thread),
                ROLE_PRODUCER,
                stack_pointer,
                DEMO_STACK_SIZE as _,
                10, // Lower priority (higher number = lower priority)
                10,
                threadx_sys::TX_NO_TIME_SLICE,
                threadx_sys::TX_AUTO_START,
            );
            if res != threadx_sys::TX_SUCCESS {
                panic!("Failed to create thread: {}", res);
            }
            thread.assume_init_mut()
        }
    };
    println_uart!(
        "Thread spawned: producer (pri=10) @ {:p}",
        thread0 as *const _
    );

    let thread1 = {
        let mut stack_pointer = core::ptr::null_mut();
        unsafe {
            let res = threadx_sys::_tx_byte_allocate(
                byte_pool,
                &mut stack_pointer,
                DEMO_STACK_SIZE as _,
                threadx_sys::TX_NO_WAIT,
            );
            if res != threadx_sys::TX_SUCCESS {
                panic!("Failed to allocate stack: {}", res);
            }
        }
        if stack_pointer.is_null() {
            panic!("Stack allocation returned null pointer");
        }

        static THREAD_STORAGE2: StaticCell<threadx_sys::TX_THREAD> = StaticCell::new();
        let thread = THREAD_STORAGE2.uninit();
        unsafe {
            let res = threadx_sys::_tx_thread_create(
                thread.as_mut_ptr(),
                core::ptr::addr_of_mut!(THREAD1_NAME) as *mut threadx_sys::CHAR,
                Some(my_thread),
                ROLE_CONSUMER,
                stack_pointer,
                DEMO_STACK_SIZE as _,
                5, // Higher priority (lower number = higher priority)
                5,
                threadx_sys::TX_NO_TIME_SLICE,
                threadx_sys::TX_AUTO_START,
            );
            if res != threadx_sys::TX_SUCCESS {
                panic!("Failed to create thread: {}", res);
            }
            thread.assume_init_mut()
        }
    };
    println_uart!(
        "Thread spawned: consumer (pri=5) @ {:p}",
        thread1 as *const _
    );

    // Deterministic marker for smoke test validation.
    // This indicates all threads have been created and the scheduler is about to start.
    // The xtask smoke test monitors for this marker to validate successful boot.
    println_uart!("APP_READY");
}

/// Thread entry point implementing real RTOS patterns.
///
/// Both threads share this entry. The `role` parameter selects behavior:
///   0 = producer: sends messages, holds mutex
///   1 = consumer: receives messages, verifies mutex protection
///
/// Scenario flow:
///   1. SCHED_RUNNING / TICK_OK  - scheduler and timer validation
///   2. Producer-consumer pipeline via queue (PIPE_OK)
///   3. Mutex contention with priority inheritance (MTX_HELD, MTX_OK)
extern "C" fn my_thread(role: u32) {
    let is_producer = role == 0;
    let name = if is_producer { "producer" } else { "consumer" };

    // Marker: Scheduler dispatched this thread
    println_uart!("SCHED_RUNNING: {}", name);

    // Brief sleep to validate timer/interrupt subsystem
    unsafe {
        threadx_sys::_tx_thread_sleep(10);
    }
    println_uart!("TICK_OK: {} resumed after sleep", name);

    // Load shared primitive pointers
    let queue = QUEUE_PTR.load(core::sync::atomic::Ordering::Acquire);
    let mutex = MUTEX_PTR.load(core::sync::atomic::Ordering::Acquire);
    let flags = FLAGS_PTR.load(core::sync::atomic::Ordering::Acquire);
    if queue.is_null() || mutex.is_null() || flags.is_null() {
        panic!("{}: primitive pointer is null - init order broken", name);
    }

    // Event flag bits for Scenario C coordination
    const FLAG_MTX_START: u32 = 0x1;
    const FLAG_MTX_HELD: u32 = 0x2;

    // ============================================================
    // Scenario B: Producer-Consumer Pipeline
    // The #1 RTOS pattern. Producer sends PIPELINE_DEPTH messages,
    // consumer receives and verifies FIFO ordering + data integrity.
    // Consumer (pri=5) preempts producer (pri=10) on each send,
    // proving priority-based scheduling.
    // ============================================================

    if is_producer {
        for i in 1..=PIPELINE_DEPTH {
            let res = unsafe {
                threadx_sys::_tx_queue_send(
                    queue,
                    &i as *const u32 as *mut core::ffi::c_void,
                    threadx_sys::TX_WAIT_FOREVER,
                )
            };
            if res != threadx_sys::TX_SUCCESS {
                panic!("producer: queue send failed (msg {}): {}", i, res);
            }
            println_uart!("producer: sent {}", i);
        }
    } else {
        for expected in 1..=PIPELINE_DEPTH {
            let mut received: u32 = 0;
            let res = unsafe {
                threadx_sys::_tx_queue_receive(
                    queue,
                    &mut received as *mut u32 as *mut core::ffi::c_void,
                    threadx_sys::TX_WAIT_FOREVER,
                )
            };
            if res != threadx_sys::TX_SUCCESS {
                panic!("consumer: queue recv failed (msg {}): {}", expected, res);
            }
            if received != expected {
                panic!(
                    "consumer: FIFO broken - expected {}, got {}",
                    expected, received
                );
            }
            println_uart!("consumer: recv {}", received);
        }
        println_uart!("PIPE_OK");
    }

    // ============================================================
    // Scenario C: Mutex Contention + Priority Inheritance
    // The #2 RTOS pattern. Shared resource protection under contention.
    //
    // Protocol (coordinated via event flags):
    //   1. Consumer signals FLAG_MTX_START, blocks on FLAG_MTX_HELD
    //   2. Producer gets signal, acquires mutex, signals FLAG_MTX_HELD
    //   3. Consumer wakes, tries mutex - blocks (producer holds it)
    //   4. ThreadX boosts producer priority (inheritance)
    //   5. Producer increments shared counter N times, releases mutex
    //   6. Consumer gets mutex, verifies counter == N
    //
    // Uses load-store pattern (not fetch_add) to detect broken
    // mutual exclusion: if two threads interleave load-store,
    // the final count will be wrong.
    // ============================================================

    if is_producer {
        // Wait for consumer to signal readiness
        let mut actual: u32 = 0;
        let res = unsafe {
            threadx_sys::_tx_event_flags_get(
                flags,
                FLAG_MTX_START,
                threadx_sys::TX_AND_CLEAR,
                &mut actual,
                threadx_sys::TX_WAIT_FOREVER,
            )
        };
        if res != threadx_sys::TX_SUCCESS {
            panic!("producer: wait for MTX_START failed: {}", res);
        }

        // Acquire mutex
        let res = unsafe { threadx_sys::_tx_mutex_get(mutex, threadx_sys::TX_WAIT_FOREVER) };
        if res != threadx_sys::TX_SUCCESS {
            panic!("producer: mutex get failed: {}", res);
        }
        println_uart!("MTX_HELD");

        // Signal consumer that mutex is held (consumer will try to acquire and block)
        let res =
            unsafe { threadx_sys::_tx_event_flags_set(flags, FLAG_MTX_HELD, threadx_sys::TX_OR) };
        if res != threadx_sys::TX_SUCCESS {
            panic!("producer: set FLAG_MTX_HELD failed: {}", res);
        }

        // Small sleep to let consumer attempt (and block on) the mutex.
        // This triggers priority inheritance: ThreadX boosts producer to
        // consumer's priority (5) because consumer is waiting on our mutex.
        unsafe {
            threadx_sys::_tx_thread_sleep(5);
        }

        // Increment shared counter N times using load-store pattern.
        // If mutual exclusion is broken, interleaved load-stores produce wrong count.
        for _ in 0..MUTEX_ROUNDS {
            let val = SHARED_COUNTER.load(core::sync::atomic::Ordering::Relaxed);
            SHARED_COUNTER.store(val + 1, core::sync::atomic::Ordering::Relaxed);
        }

        // Release mutex
        let res = unsafe { threadx_sys::_tx_mutex_put(mutex) };
        if res != threadx_sys::TX_SUCCESS {
            panic!("producer: mutex put failed: {}", res);
        }
    } else {
        // Signal producer to start mutex test
        let res =
            unsafe { threadx_sys::_tx_event_flags_set(flags, FLAG_MTX_START, threadx_sys::TX_OR) };
        if res != threadx_sys::TX_SUCCESS {
            panic!("consumer: set FLAG_MTX_START failed: {}", res);
        }

        // Wait for producer to hold the mutex
        let mut actual: u32 = 0;
        let res = unsafe {
            threadx_sys::_tx_event_flags_get(
                flags,
                FLAG_MTX_HELD,
                threadx_sys::TX_AND_CLEAR,
                &mut actual,
                threadx_sys::TX_WAIT_FOREVER,
            )
        };
        if res != threadx_sys::TX_SUCCESS {
            panic!("consumer: wait for MTX_HELD failed: {}", res);
        }

        // Try to acquire mutex - this blocks because producer holds it.
        // ThreadX will boost producer's priority to ours (priority inheritance).
        let res = unsafe { threadx_sys::_tx_mutex_get(mutex, threadx_sys::TX_WAIT_FOREVER) };
        if res != threadx_sys::TX_SUCCESS {
            panic!("consumer: mutex get failed: {}", res);
        }

        // Verify shared counter integrity
        let count = SHARED_COUNTER.load(core::sync::atomic::Ordering::Relaxed);
        if count != MUTEX_ROUNDS {
            panic!(
                "consumer: counter mismatch - expected {}, got {} (mutual exclusion broken)",
                MUTEX_ROUNDS, count
            );
        }

        let res = unsafe { threadx_sys::_tx_mutex_put(mutex) };
        if res != threadx_sys::TX_SUCCESS {
            panic!("consumer: mutex put failed: {}", res);
        }

        println_uart!("MTX_OK");
    }

    // Park thread
    loop {
        unsafe {
            threadx_sys::_tx_thread_sleep(1000);
        }
    }
}

/// Entry point for the Rust application.
///
/// This is invoked by the startup code in 'startup.rs'.
#[no_mangle]
pub extern "C" fn kmain() {
    // Check if this is a re-entry (system reset)
    static KMAIN_ENTRY_COUNT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
    let entry_count = KMAIN_ENTRY_COUNT.fetch_add(1, core::sync::atomic::Ordering::SeqCst);

    // Initialize UART hardware
    let mut uart0 = unsafe { Uart::new_uart0() };
    _ = writeln!(uart0, "Running ThreadX... (entry #{})", entry_count);

    // Initialize buffered UART system (interrupt-driven)
    unsafe {
        cortex_r5_sample::buffered_uart::init();
    }

    let mut timer0 = unsafe { Timer0::new_timer0() };
    timer0.init(
        10_000,
        sp804_timer::Mode::AutoReload,
        sp804_timer::Interrupts::Enabled,
    );

    // Enable interrupts on the VIC
    let mut vic = unsafe { pl190_vic::Interrupt::new() };
    vic.init();
    vic.enable_interrupt(TIMER0_IRQ);

    // Debug: Check VIC state after Timer0
    let vic_inten_before = unsafe { ((VIC_BASE + 0x10) as *const u32).read_volatile() };
    _ = writeln!(
        uart0,
        "VIC INTENABLE after Timer0: 0x{:08x}",
        vic_inten_before
    );

    vic.enable_interrupt(UART0_IRQ);

    // Debug: Check VIC state after UART0
    let vic_inten_after = unsafe { ((VIC_BASE + 0x10) as *const u32).read_volatile() };
    _ = writeln!(
        uart0,
        "VIC INTENABLE after UART0: 0x{:08x}",
        vic_inten_after
    );

    timer0.start();

    unsafe {
        threadx_sys::_tx_initialize_kernel_enter();
    }

    panic!("Kernel exited");
}

/// Called from the main interrupt handler
#[no_mangle]
unsafe extern "C" fn handle_interrupt() {
    extern "C" {
        fn _tx_timer_interrupt();
    }

    // Check for Timer0 interrupt
    if Timer0::is_pending() {
        unsafe {
            _tx_timer_interrupt();
        }
        Timer0::clear_interrupt();
    }

    // Check for UART TX interrupt on VIC
    // Note: We check the VIC status register to determine which interrupt fired
    let vic_status = unsafe {
        let vic_base = VIC_BASE as *const u32;
        vic_base.read_volatile()
    };

    if (vic_status & (1 << UART0_IRQ)) != 0 {
        cortex_r5_sample::buffered_uart::handle_tx_interrupt();
    }
}

/// Handles unrecoverable `panic!` calls in the application.
///
/// Outputs the panic details to the console and exits QEMU using a semihosting
/// breakpoint.
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    use core::fmt::Write;

    // Use emergency UART bypass to avoid mutex deadlocks during panic
    struct EmergencyWriter;
    impl core::fmt::Write for EmergencyWriter {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            unsafe {
                Uart::<UART0_BASE>::emergency_write_str(s);
            }
            Ok(())
        }
    }

    let mut w = EmergencyWriter;
    let _ = writeln!(w, "\r\nPANIC: {:?}", info);

    const SYS_REPORTEXC: u32 = 0x18;
    loop {
        // Exit, using semihosting
        unsafe {
            core::arch::asm!(
                "svc 0x123456",
                in("r0") SYS_REPORTEXC,
                in("r1") 0x20026
            )
        }
    }
}
