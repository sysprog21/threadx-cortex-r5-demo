//! Rust demonstration for a QEMU Arm Cortex-R machine running ThreadX.

#![no_std]
#![no_main]

use byte_strings::c;
use core::{cell::UnsafeCell, fmt::Write as _, mem::MaybeUninit};
use cortex_r5_sample::{
    pl011_uart::Uart,
    pl190_vic,
    sp804_timer::{self, Timer0},
};
use static_cell::StaticCell;

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

static UART: GlobalUart = GlobalUart::new();

unsafe impl Sync for GlobalUart {}

struct GlobalUart {
    inner: UnsafeCell<Option<Uart<0x101f_1000>>>,
    mutex: MaybeUninit<UnsafeCell<threadx_sys::TX_MUTEX_STRUCT>>,
}

impl GlobalUart {
    /// Create a new, empty, global UART wrapper
    const fn new() -> GlobalUart {
        GlobalUart {
            inner: UnsafeCell::new(None),
            mutex: MaybeUninit::uninit(),
        }
    }

    /// Registers a new UART at runtime and initializes the associated ThreadX
    /// mutex.
    ///
    /// # Safety
    /// This must only be called during initialization, before any threads are
    /// running, and it should be invoked only once.
    unsafe fn store(&self, uart: Uart<0x101f_1000>) {
        // Init the ThreadX mutex
        unsafe {
            // init mutex
            threadx_sys::_tx_mutex_create(
                UnsafeCell::raw_get(self.mutex.as_ptr()),
                "my_mutex\0".as_ptr() as _,
                0,
            );
            // unsafely store UART object
            let ptr = self.inner.get();
            let mut_ret = &mut *ptr;
            *mut_ret = Some(uart);
        }
    }
}

// This implementation is for '&GlobalUart', allowing writes to a shared
// reference without requiring an exclusive mutable reference.
impl core::fmt::Write for &GlobalUart {
    /// Write the string to the inner UART, with a lock held
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        // Grab ThreadX mutex
        unsafe {
            threadx_sys::_tx_mutex_get(
                UnsafeCell::raw_get(self.mutex.as_ptr()),
                threadx_sys::TX_WAIT_FOREVER,
            );
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Acquire);
        }

        // # Safety
        // The ThreadX mutex is held at this point.
        let uart_option_ref = unsafe { &mut *self.inner.get() };
        let Some(uart) = uart_option_ref else {
            return Err(core::fmt::Error);
        };

        let result = uart.write_str(s);

        // Release the UART reference, followed by the ThreadX mutex.
        let _ = uart;
        unsafe {
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);
            threadx_sys::_tx_mutex_put(UnsafeCell::raw_get(self.mutex.as_ptr()));
        }

        result
    }
}

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
/// - Validates pointers before dereferencing to avoid double-faults
/// - Halts CPU with interrupts disabled instead of panic (no unwinding)
#[no_mangle]
extern "C" fn stack_error_handler(thread_ptr: *mut threadx_sys::TX_THREAD_STRUCT) {
    use core::fmt::Write;

    // Emergency writer that bypasses all mutexes
    struct EmergencyWriter;
    impl core::fmt::Write for EmergencyWriter {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            unsafe {
                Uart::<0x101f_1000>::emergency_write_str(s);
            }
            Ok(())
        }
    }

    let mut w = EmergencyWriter;

    unsafe {
        let _ = writeln!(w, "\r\n!!! STACK OVERFLOW DETECTED !!!");

        // Validate pointer before dereferencing
        if thread_ptr.is_null() {
            let _ = writeln!(w, "Thread: NULL pointer");
        } else {
            // Print pointer first in case struct is corrupted
            let _ = writeln!(w, "Thread @ {:p}", thread_ptr);

            let thread = &*thread_ptr;

            // Attempt to read name carefully (may be corrupted)
            let name_ptr = thread.tx_thread_name;
            if !name_ptr.is_null() {
                // Limit read to 32 bytes to avoid reading garbage forever
                let bytes = core::slice::from_raw_parts(name_ptr as *const u8, 32);
                let _ = write!(w, "Name: ");
                for &b in bytes {
                    if b == 0 {
                        break;
                    }
                    if b.is_ascii_graphic() || b == b' ' {
                        let _ = write!(w, "{}", b as char);
                    } else {
                        let _ = write!(w, "?");
                    }
                }
                let _ = writeln!(w, "");
            }

            // Print stack boundaries
            let _ = writeln!(
                w,
                "Stack: {:p} - {:p} (size={})",
                thread.tx_thread_stack_start,
                thread.tx_thread_stack_end,
                thread.tx_thread_stack_size
            );
            let _ = writeln!(w, "Stack SP: {:p}", thread.tx_thread_stack_ptr);
        }

        let _ = writeln!(w, "System halted.");

        // Disable interrupts and halt CPU
        // Do NOT use panic!() - it may try to acquire locks
        core::arch::asm!(
            "cpsid i",      // Disable interrupts (CPSR I bit)
            "2:",
            "wfi",          // Wait for interrupt (low power)
            "b 2b",         // Loop forever
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
    _ = writeln!(&UART, "In tx_application_define()...");

    // Register stack error notification handler
    unsafe {
        threadx_sys::_tx_thread_stack_error_notify(Some(stack_error_handler));
    }

    // ThreadX requires a mutable pointer to a character array for object names,
    // which it retains internally. These names must have a static lifetime.
    // To satisfy the API, we cast away the constness of a static string slice.

    let byte_pool = {
        static BYTE_POOL: StaticCell<threadx_sys::TX_BYTE_POOL> = StaticCell::new();
        static BYTE_POOL_STORAGE: StaticCell<[u8; DEMO_POOL_SIZE]> = StaticCell::new();
        let byte_pool = BYTE_POOL.uninit();
        let byte_pool_storage = BYTE_POOL_STORAGE.uninit();
        unsafe {
            threadx_sys::_tx_byte_pool_create(
                byte_pool.as_mut_ptr(),
                c!("byte-pool0").as_ptr() as *mut threadx_sys::CHAR,
                byte_pool_storage.as_mut_ptr() as *mut _,
                DEMO_POOL_SIZE as u32,
            );
            byte_pool.assume_init_mut()
        }
    };

    let entry = 0x12345678;
    let thread0 = {
        let mut stack_pointer = core::ptr::null_mut();
        unsafe {
            threadx_sys::_tx_byte_allocate(
                byte_pool,
                &mut stack_pointer,
                DEMO_STACK_SIZE as _,
                threadx_sys::TX_NO_WAIT,
            );
        }
        _ = writeln!(&UART, "Stack allocated @ {:p}", stack_pointer);
        if stack_pointer.is_null() {
            panic!("No space for stack");
        }

        static THREAD_STORAGE: StaticCell<threadx_sys::TX_THREAD> = StaticCell::new();
        let thread = THREAD_STORAGE.uninit();
        unsafe {
            let res = threadx_sys::_tx_thread_create(
                thread.as_mut_ptr(),
                c!("thread0").as_ptr() as *mut threadx_sys::CHAR,
                Some(my_thread),
                entry,
                stack_pointer,
                DEMO_STACK_SIZE as _,
                1,
                1,
                threadx_sys::TX_NO_TIME_SLICE,
                threadx_sys::TX_AUTO_START,
            );
            if res != threadx_sys::TX_SUCCESS {
                panic!("Failed to create thread: {}", res);
            }
            thread.assume_init_mut()
        }
    };
    _ = writeln!(
        &UART,
        "Thread spawned (entry={:08x}) @ {:p}",
        entry, thread0 as *const _
    );

    let entry = 0xAABBCCDD;
    let thread1 = {
        let mut stack_pointer = core::ptr::null_mut();
        unsafe {
            threadx_sys::_tx_byte_allocate(
                byte_pool,
                &mut stack_pointer,
                DEMO_STACK_SIZE as _,
                threadx_sys::TX_NO_WAIT,
            );
        }
        _ = writeln!(&UART, "Stack allocated @ {:p}", stack_pointer);
        if stack_pointer.is_null() {
            panic!("No space for stack");
        }

        static THREAD_STORAGE2: StaticCell<threadx_sys::TX_THREAD> = StaticCell::new();
        let thread = THREAD_STORAGE2.uninit();
        unsafe {
            let res = threadx_sys::_tx_thread_create(
                thread.as_mut_ptr(),
                c!("thread1").as_ptr() as *mut threadx_sys::CHAR,
                Some(my_thread),
                entry,
                stack_pointer,
                DEMO_STACK_SIZE as _,
                1,
                1,
                threadx_sys::TX_NO_TIME_SLICE,
                threadx_sys::TX_AUTO_START,
            );
            if res != threadx_sys::TX_SUCCESS {
                panic!("Failed to create thread: {}", res);
            }
            thread.assume_init_mut()
        }
    };
    _ = writeln!(
        &UART,
        "Thread spawned (entry={:08x}) @ {:p}",
        entry, thread1 as *const _
    );
}

extern "C" fn my_thread(value: u32) {
    _ = writeln!(&UART, "Thread({:08x})", value);
    let mut thread_counter = 0;
    loop {
        thread_counter += 1;

        unsafe {
            threadx_sys::_tx_thread_sleep(100);
        }

        _ = writeln!(&UART, "Thread({:08x}), count = {}", value, thread_counter);
    }
}

/// Entry point for the Rust application.
///
/// This is invoked by the startup code in 'lib.rs'.
#[no_mangle]
pub extern "C" fn kmain() {
    // Create a UART
    let mut uart0 = unsafe { Uart::new_uart0() };
    _ = writeln!(uart0, "Running ThreadX...");
    unsafe {
        UART.store(uart0);
    }

    let mut timer0 = unsafe { Timer0::new_timer0() };
    timer0.init(
        10_000,
        sp804_timer::Mode::AutoReload,
        sp804_timer::Interrupts::Enabled,
    );

    // Enable the Timer0 interrupt and associate it with the IRQ on this core.
    // It corresponds to PIC interrupt 4.
    let mut vic = unsafe { pl190_vic::Interrupt::new() };
    vic.init();
    vic.enable_interrupt(4);

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

    if Timer0::is_pending() {
        unsafe {
            _tx_timer_interrupt();
        }
        Timer0::clear_interrupt();
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
                Uart::<0x101f_1000>::emergency_write_str(s);
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
