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
    static mut THREAD0_NAME: [u8; 16] = *b"thread0\0\0\0\0\0\0\0\0\0";
    static mut THREAD1_NAME: [u8; 16] = *b"thread1\0\0\0\0\0\0\0\0\0";

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

    let entry = 0x12345678;
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
        println_uart!("Stack allocated @ {:p}", stack_pointer);
        // Double-check: even on success, verify pointer is valid
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
    println_uart!(
        "Thread spawned (entry={:08x}) @ {:p}",
        entry,
        thread0 as *const _
    );

    let entry = 0xAABBCCDD;
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
        println_uart!("Stack allocated @ {:p}", stack_pointer);
        // Double-check: even on success, verify pointer is valid
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
    println_uart!(
        "Thread spawned (entry={:08x}) @ {:p}",
        entry,
        thread1 as *const _
    );
}

extern "C" fn my_thread(value: u32) {
    unsafe {
        threadx_sys::_tx_thread_sleep(10);
    }

    println_uart!("Thread({:08x}) started", value);

    let mut thread_counter = 0;
    loop {
        thread_counter += 1;

        unsafe {
            threadx_sys::_tx_thread_sleep(100);
        }

        println_uart!("Thread({:08x}), count = {}", value, thread_counter);
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
