//! Rust demonstration for a QEMU Arm Cortex-R machine running ThreadX.
//!
//! This binary is not compiled when the `host-test` feature is enabled.

// Guard entire binary for embedded target only
#![cfg(not(feature = "host-test"))]
#![no_std]
#![no_main]

use cortex_r5_sample::{
    config::{TIMER0_IRQ, UART0_BASE, UART0_IRQ, VIC_BASE},
    pl011_uart::Uart,
    pl190_vic,
    sp804_timer::{self, Timer0},
};
use static_cell::StaticCell;

// Import safe UART logging macros
use cortex_r5_sample::println_uart;
use cortex_r5_sample::spsc::Spsc;

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

// Scenario D: Lock-free SPSC ring depth (messages pushed by producer)
const SPSC_DEPTH: u32 = 8;

// Lock-free SPSC ring for ISR-to-thread messaging demo.
// In production, the ISR side would be a real interrupt handler pushing
// sensor data, UART RX bytes, or DMA completion tokens. Here we demonstrate
// the same pattern thread-to-thread since the producer API is identical.
static SPSC_RING: Spsc<u32, 8> = Spsc::new();

// Global primitives for runtime validation (initialized in tx_application_define)
static TEST_MUTEX: StaticCell<threadx_sys::TX_MUTEX> = StaticCell::new();
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

/// Initializes the application.
///
/// This is invoked by ThreadX during scheduler start-up and is used to create
/// threads.
#[no_mangle]
extern "C" fn tx_application_define(_first_unused_memory: *mut core::ffi::c_void) {
    println_uart!("In tx_application_define()...");

    // ThreadX requires a mutable pointer to a character array for object names,
    // which it retains internally. Use mutable static buffers to avoid casting
    // read-only string literals to *mut CHAR (which would be UB if ThreadX writes).
    // Use addr_of_mut! to get raw pointers without creating mutable references.
    static mut BYTE_POOL_NAME: [u8; 16] = *b"byte-pool0\0\0\0\0\0\0";
    static mut THREAD0_NAME: [u8; 16] = *b"producer\0\0\0\0\0\0\0\0";
    static mut THREAD1_NAME: [u8; 16] = *b"consumer\0\0\0\0\0\0\0\0";
    static mut MUTEX_NAME: [u8; 16] = *b"test-mutex\0\0\0\0\0\0";
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
                panic!("byte pool create failed");
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
            panic!("mutex create failed");
        }
        let mutex = mutex.assume_init_mut();
        MUTEX_PTR.store(mutex, core::sync::atomic::Ordering::Release);
    }

    // Event flags (coordination between threads)
    unsafe {
        let flags = TEST_FLAGS.uninit();
        let res = threadx_sys::_tx_event_flags_create(
            flags.as_mut_ptr(),
            core::ptr::addr_of_mut!(FLAGS_NAME) as *mut threadx_sys::CHAR,
        );
        if res != threadx_sys::TX_SUCCESS {
            panic!("event flags create failed");
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
            panic!("queue create failed");
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
                panic!("stack alloc failed");
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
                panic!("thread create failed");
            }
            thread.assume_init_mut()
        }
    };
    println_uart!(
        "Thread spawned: producer (pri=10) @ 0x{:08x}",
        thread0 as *const _ as usize
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
                panic!("stack alloc failed");
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
                panic!("thread create failed");
            }
            thread.assume_init_mut()
        }
    };
    println_uart!(
        "Thread spawned: consumer (pri=5) @ 0x{:08x}",
        thread1 as *const _ as usize
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
    println_uart!("TICK_OK: resumed after sleep");

    // Load shared primitive pointers
    let queue = QUEUE_PTR.load(core::sync::atomic::Ordering::Acquire);
    let mutex = MUTEX_PTR.load(core::sync::atomic::Ordering::Acquire);
    let flags = FLAGS_PTR.load(core::sync::atomic::Ordering::Acquire);
    if queue.is_null() || mutex.is_null() || flags.is_null() {
        panic!("primitive pointer null");
    }

    // Event flag bits for Scenario C coordination
    const FLAG_MTX_START: u32 = 0x1;
    const FLAG_MTX_HELD: u32 = 0x2;
    // Event flag bit for Scenario D (SPSC data ready)
    const FLAG_SPSC_DATA: u32 = 0x4;

    // ============================================================
    // Scenario B: Producer-Consumer Pipeline
    // The #1 RTOS pattern. Producer sends PIPELINE_DEPTH messages,
    // consumer receives and verifies FIFO ordering + data integrity.
    // Consumer (pri=5) preempts producer (pri=10) on each send,
    // proving priority-based scheduling.
    // ============================================================

    if is_producer {
        for i in 1..PIPELINE_DEPTH + 1 {
            let res = unsafe {
                threadx_sys::_tx_queue_send(
                    queue,
                    &i as *const u32 as *mut core::ffi::c_void,
                    threadx_sys::TX_WAIT_FOREVER,
                )
            };
            if res != threadx_sys::TX_SUCCESS {
                panic!("queue send failed");
            }
            println_uart!("producer: sent {}", i);
        }
    } else {
        for expected in 1..PIPELINE_DEPTH + 1 {
            let mut received: u32 = 0;
            let res = unsafe {
                threadx_sys::_tx_queue_receive(
                    queue,
                    &mut received as *mut u32 as *mut core::ffi::c_void,
                    threadx_sys::TX_WAIT_FOREVER,
                )
            };
            if res != threadx_sys::TX_SUCCESS {
                panic!("queue recv failed");
            }
            if received != expected {
                panic!("FIFO order broken");
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
            panic!("wait MTX_START failed");
        }

        // Acquire mutex
        let res = unsafe { threadx_sys::_tx_mutex_get(mutex, threadx_sys::TX_WAIT_FOREVER) };
        if res != threadx_sys::TX_SUCCESS {
            panic!("producer mutex get failed");
        }
        println_uart!("MTX_HELD");

        // Signal consumer that mutex is held (consumer will try to acquire and block)
        let res =
            unsafe { threadx_sys::_tx_event_flags_set(flags, FLAG_MTX_HELD, threadx_sys::TX_OR) };
        if res != threadx_sys::TX_SUCCESS {
            panic!("set MTX_HELD failed");
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
            panic!("producer mutex put failed");
        }
    } else {
        // Signal producer to start mutex test
        let res =
            unsafe { threadx_sys::_tx_event_flags_set(flags, FLAG_MTX_START, threadx_sys::TX_OR) };
        if res != threadx_sys::TX_SUCCESS {
            panic!("set MTX_START failed");
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
            panic!("wait MTX_HELD failed");
        }

        // Try to acquire mutex - this blocks because producer holds it.
        // ThreadX will boost producer's priority to ours (priority inheritance).
        let res = unsafe { threadx_sys::_tx_mutex_get(mutex, threadx_sys::TX_WAIT_FOREVER) };
        if res != threadx_sys::TX_SUCCESS {
            panic!("consumer mutex get failed");
        }

        // Verify shared counter integrity
        let count = SHARED_COUNTER.load(core::sync::atomic::Ordering::Relaxed);
        if count != MUTEX_ROUNDS {
            panic!("counter mismatch - mutual exclusion broken");
        }

        let res = unsafe { threadx_sys::_tx_mutex_put(mutex) };
        if res != threadx_sys::TX_SUCCESS {
            panic!("consumer mutex put failed");
        }

        println_uart!("MTX_OK");
    }

    // ============================================================
    // Scenario D: Lock-Free SPSC Ring + Event Flags
    // The canonical ISR-to-thread messaging pattern. Producer pushes
    // data into a lock-free ring and signals via event flags. Consumer
    // waits on the flag, then drains the ring without any locks.
    //
    // The producer side is ISR-safe: no blocking, no critical sections,
    // just an atomic index update. Here we demonstrate thread-to-thread
    // to validate the data path; the producer code is identical in ISR.
    //
    // Flow:
    //   1. Consumer (pri=5) reaches here first, blocks on FLAG_SPSC_DATA
    //   2. Producer (pri=10) pushes SPSC_DEPTH values, sets FLAG_SPSC_DATA
    //   3. Consumer preempts, drains ring, verifies data integrity
    // ============================================================

    if is_producer {
        for i in 1..=SPSC_DEPTH {
            // push() is lock-free and ISR-safe (no critical section needed)
            let res = unsafe { SPSC_RING.push(i) };
            if res.is_err() {
                panic!("spsc push failed: ring full");
            }
        }

        // Signal consumer that data is ready (tx_event_flags_set is ISR-safe)
        let res =
            unsafe { threadx_sys::_tx_event_flags_set(flags, FLAG_SPSC_DATA, threadx_sys::TX_OR) };
        if res != threadx_sys::TX_SUCCESS {
            panic!("set SPSC_DATA flag failed");
        }
        println_uart!("producer: spsc sent {} values", SPSC_DEPTH);
    } else {
        // Wait for producer to signal data is ready
        let mut actual: u32 = 0;
        let res = unsafe {
            threadx_sys::_tx_event_flags_get(
                flags,
                FLAG_SPSC_DATA,
                threadx_sys::TX_AND_CLEAR,
                &mut actual,
                threadx_sys::TX_WAIT_FOREVER,
            )
        };
        if res != threadx_sys::TX_SUCCESS {
            panic!("wait SPSC_DATA flag failed");
        }

        // Drain the ring - pop until empty
        let mut count: u32 = 0;
        let mut sum: u32 = 0;
        while let Some(val) = unsafe { SPSC_RING.pop() } {
            count += 1;
            sum += val;
        }

        // Verify: received all values and data integrity (1+2+...+N = N*(N+1)/2)
        let expected_sum = SPSC_DEPTH * (SPSC_DEPTH + 1) / 2;
        if count != SPSC_DEPTH || sum != expected_sum {
            panic!("spsc data mismatch");
        }
        println_uart!("SPSC_OK");
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
    let _ = ufmt::uwriteln!(uart0, "Running ThreadX... (entry #{})", entry_count);

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
    let _ = ufmt::uwriteln!(
        uart0,
        "VIC INTENABLE after Timer0: 0x{:08x}",
        vic_inten_before
    );

    vic.enable_interrupt(UART0_IRQ);

    // Debug: Check VIC state after UART0
    let vic_inten_after = unsafe { ((VIC_BASE + 0x10) as *const u32).read_volatile() };
    let _ = ufmt::uwriteln!(
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
/// breakpoint. Uses emergency UART bypass (no core::fmt dependency).
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    /// Write a u32 as decimal digits via emergency UART (no core::fmt).
    unsafe fn emergency_write_u32(n: u32) {
        if n == 0 {
            Uart::<UART0_BASE>::emergency_write_str("0");
            return;
        }
        let mut buf = [0u8; 10];
        let mut pos = 10usize;
        let mut val = n;
        while val > 0 {
            pos -= 1;
            buf[pos] = b'0' + (val % 10) as u8;
            val /= 10;
        }
        Uart::<UART0_BASE>::emergency_write_str(core::str::from_utf8_unchecked(&buf[pos..]));
    }

    unsafe {
        Uart::<UART0_BASE>::emergency_write_str("\r\nPANIC: ");

        if let Some(loc) = info.location() {
            Uart::<UART0_BASE>::emergency_write_str(loc.file());
            Uart::<UART0_BASE>::emergency_write_str(":");
            emergency_write_u32(loc.line());
        }

        // All panics use static strings, so as_str() returns Some
        if let Some(msg) = info.message().as_str() {
            Uart::<UART0_BASE>::emergency_write_str(": ");
            Uart::<UART0_BASE>::emergency_write_str(msg);
        }

        Uart::<UART0_BASE>::emergency_write_str("\r\n");
    }

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
