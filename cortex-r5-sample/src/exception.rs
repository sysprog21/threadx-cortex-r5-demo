//! ARM Cortex-R5 Exception Handlers
//!
//! Provides diagnostic fault handlers for hardware exceptions that occur outside
//! Rust's panic machinery (misaligned access, invalid instructions, MMU violations).

use crate::config::UART0_BASE;

/// Helper: Calculate length of C string with safety bound
///
/// Prevents infinite loops if string is corrupted/non-terminated
#[inline]
unsafe fn c_strlen_safe(s: *const u8, max: usize) -> usize {
    let mut len = 0;
    while len < max && *s.add(len) != 0 {
        len += 1;
    }
    len
}

/// RAM range check for basic sanity validation
/// QEMU VersatileAB has 16MB RAM starting at 0x0
const RAM_START: usize = 0x0000_0000;
const RAM_END: usize = 0x0100_0000;

/// Helper: Format hex value into buffer
///
/// Returns formatted string slice on success
fn format_hex<'a>(buf: &'a mut [u8], prefix: &str, value: u32) -> Result<&'a str, ()> {
    use core::fmt::Write;

    struct BufWriter<'a> {
        buf: &'a mut [u8],
        pos: usize,
    }

    impl<'a> Write for BufWriter<'a> {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            let bytes = s.as_bytes();
            if self.pos + bytes.len() > self.buf.len() {
                return Err(core::fmt::Error);
            }
            self.buf[self.pos..self.pos + bytes.len()].copy_from_slice(bytes);
            self.pos += bytes.len();
            Ok(())
        }
    }

    let mut writer = BufWriter { buf, pos: 0 };
    if writeln!(writer, "{}{:08X}", prefix, value).is_ok() {
        core::str::from_utf8(&writer.buf[..writer.pos]).map_err(|_| ())
    } else {
        Err(())
    }
}

/// Data Fault Address Register (CP15 c6, c0, 0)
#[inline(always)]
fn read_dfar() -> u32 {
    let value: u32;
    unsafe {
        core::arch::asm!(
            "mrc p15, 0, {}, c6, c0, 0",
            out(reg) value,
            options(nostack, nomem)
        );
    }
    value
}

/// Data Fault Status Register (CP15 c5, c0, 0)
#[inline(always)]
fn read_dfsr() -> u32 {
    let value: u32;
    unsafe {
        core::arch::asm!(
            "mrc p15, 0, {}, c5, c0, 0",
            out(reg) value,
            options(nostack, nomem)
        );
    }
    value
}

/// Instruction Fault Address Register (CP15 c6, c0, 2)
#[inline(always)]
fn read_ifar() -> u32 {
    let value: u32;
    unsafe {
        core::arch::asm!(
            "mrc p15, 0, {}, c6, c0, 2",
            out(reg) value,
            options(nostack, nomem)
        );
    }
    value
}

/// Instruction Fault Status Register (CP15 c5, c0, 1)
#[inline(always)]
fn read_ifsr() -> u32 {
    let value: u32;
    unsafe {
        core::arch::asm!(
            "mrc p15, 0, {}, c5, c0, 1",
            out(reg) value,
            options(nostack, nomem)
        );
    }
    value
}

/// Data Abort Exception Handler
///
/// Called when a data access causes an abort (misaligned access, access to
/// invalid memory region, MPU/MMU violation).
///
/// # Parameters
/// - `pc`: Faulting instruction address (LR - 8, adjusted by assembly)
///
/// # Safety
/// This is an exception handler called directly from the vector table.
/// Must never return.
#[no_mangle]
pub extern "C" fn data_abort_handler(pc: u32) -> ! {
    let dfar = read_dfar();
    let dfsr = read_dfsr();

    // Decode fault status (bits [3:0] and bit 10)
    let status = (dfsr & 0xF) | ((dfsr & 0x400) >> 6);

    // Try to identify current thread for better diagnostics
    let thread = unsafe { threadx_sys::_tx_thread_identify() };

    // Log fault details using emergency UART (bypasses all locks)
    unsafe {
        use crate::pl011_uart::Uart;

        Uart::<UART0_BASE>::emergency_write_str("\n=== DATA ABORT ===\n");

        // Print PC first - critical diagnostic info
        let mut buf = [0u8; 64];
        if let Ok(s) = format_hex(&mut buf, "PC:            0x", pc) {
            Uart::<UART0_BASE>::emergency_write_str(s);
        }

        // Format fault address
        if let Ok(s) = format_hex(&mut buf, "Fault Address: 0x", dfar) {
            Uart::<UART0_BASE>::emergency_write_str(s);
        }

        // Format fault status
        if let Ok(s) = format_hex(&mut buf, "Fault Status:  0x", dfsr) {
            Uart::<UART0_BASE>::emergency_write_str(s);
        }
        if let Ok(s) = format_hex(&mut buf, " (type=0x", status) {
            Uart::<UART0_BASE>::emergency_write_str(s);
        }

        // Safety checks for thread pointer to avoid nested faults
        if !thread.is_null()
            && (thread as usize) >= RAM_START
            && (thread as usize) < RAM_END
            && (thread as usize & 0x3) == 0
        {
            let name_ptr = (*thread).tx_thread_name;
            if !name_ptr.is_null()
                && (name_ptr as usize) >= RAM_START
                && (name_ptr as usize) < RAM_END
            {
                Uart::<UART0_BASE>::emergency_write_str("Thread: ");
                // Bounded string read to prevent infinite loop on corrupted data
                let name_bytes = core::slice::from_raw_parts(
                    name_ptr as *const u8,
                    c_strlen_safe(name_ptr as *const u8, 32),
                );
                if let Ok(s) = core::str::from_utf8(name_bytes) {
                    Uart::<UART0_BASE>::emergency_write_str(s);
                    Uart::<UART0_BASE>::emergency_write_str("\n");
                }
            }
        } else {
            Uart::<UART0_BASE>::emergency_write_str("Thread: <none/invalid>\n");
        }

        Uart::<UART0_BASE>::emergency_write_str("==================\n\n");
    }

    // Halt - no recovery possible
    loop {
        core::hint::spin_loop();
    }
}

/// Prefetch Abort Exception Handler
///
/// Called when instruction fetch causes an abort (branch to invalid address,
/// attempt to execute data region, MPU/MMU violation on instruction fetch).
///
/// # Parameters
/// - `pc`: Faulting instruction address (LR - 4, adjusted by assembly)
///
/// # Safety
/// This is an exception handler called directly from the vector table.
/// Must never return.
#[no_mangle]
pub extern "C" fn prefetch_abort_handler(pc: u32) -> ! {
    let ifar = read_ifar();
    let ifsr = read_ifsr();

    // Decode fault status
    let status = (ifsr & 0xF) | ((ifsr & 0x400) >> 6);

    let thread = unsafe { threadx_sys::_tx_thread_identify() };

    unsafe {
        use crate::pl011_uart::Uart;

        Uart::<UART0_BASE>::emergency_write_str("\n=== PREFETCH ABORT ===\n");

        // Print PC first - critical diagnostic info
        let mut buf = [0u8; 64];
        if let Ok(s) = format_hex(&mut buf, "PC:            0x", pc) {
            Uart::<UART0_BASE>::emergency_write_str(s);
        }

        // Format fault address
        if let Ok(s) = format_hex(&mut buf, "Fault Address: 0x", ifar) {
            Uart::<UART0_BASE>::emergency_write_str(s);
        }

        // Format fault status
        if let Ok(s) = format_hex(&mut buf, "Fault Status:  0x", ifsr) {
            Uart::<UART0_BASE>::emergency_write_str(s);
        }
        if let Ok(s) = format_hex(&mut buf, " (type=0x", status) {
            Uart::<UART0_BASE>::emergency_write_str(s);
        }

        // Safety checks for thread pointer
        if !thread.is_null()
            && (thread as usize) >= RAM_START
            && (thread as usize) < RAM_END
            && (thread as usize & 0x3) == 0
        {
            let name_ptr = (*thread).tx_thread_name;
            if !name_ptr.is_null()
                && (name_ptr as usize) >= RAM_START
                && (name_ptr as usize) < RAM_END
            {
                Uart::<UART0_BASE>::emergency_write_str("Thread: ");
                let name_bytes = core::slice::from_raw_parts(
                    name_ptr as *const u8,
                    c_strlen_safe(name_ptr as *const u8, 32),
                );
                if let Ok(s) = core::str::from_utf8(name_bytes) {
                    Uart::<UART0_BASE>::emergency_write_str(s);
                    Uart::<UART0_BASE>::emergency_write_str("\n");
                }
            }
        } else {
            Uart::<UART0_BASE>::emergency_write_str("Thread: <none/invalid>\n");
        }

        Uart::<UART0_BASE>::emergency_write_str("======================\n\n");
    }

    loop {
        core::hint::spin_loop();
    }
}

/// Undefined Instruction Exception Handler
///
/// Called when the processor encounters an instruction it cannot decode
/// (corrupted code, invalid opcode, FPU instruction without FPU enabled).
///
/// # Parameters
/// - `pc`: Faulting instruction address (LR - 4, adjusted by assembly)
///
/// # Safety
/// This is an exception handler called directly from the vector table.
/// Must never return.
#[no_mangle]
pub extern "C" fn undefined_handler(pc: u32) -> ! {
    let thread = unsafe { threadx_sys::_tx_thread_identify() };

    unsafe {
        use crate::pl011_uart::Uart;

        Uart::<UART0_BASE>::emergency_write_str("\n=== UNDEFINED INSTRUCTION ===\n");

        // Print PC - critical diagnostic info
        let mut buf = [0u8; 64];
        if let Ok(s) = format_hex(&mut buf, "PC:            0x", pc) {
            Uart::<UART0_BASE>::emergency_write_str(s);
        }

        // Safety checks for thread pointer
        if !thread.is_null()
            && (thread as usize) >= RAM_START
            && (thread as usize) < RAM_END
            && (thread as usize & 0x3) == 0
        {
            let name_ptr = (*thread).tx_thread_name;
            if !name_ptr.is_null()
                && (name_ptr as usize) >= RAM_START
                && (name_ptr as usize) < RAM_END
            {
                Uart::<UART0_BASE>::emergency_write_str("Thread: ");
                let name_bytes = core::slice::from_raw_parts(
                    name_ptr as *const u8,
                    c_strlen_safe(name_ptr as *const u8, 32),
                );
                if let Ok(s) = core::str::from_utf8(name_bytes) {
                    Uart::<UART0_BASE>::emergency_write_str(s);
                    Uart::<UART0_BASE>::emergency_write_str("\n");
                }
            }
        } else {
            Uart::<UART0_BASE>::emergency_write_str("Thread: <none/invalid>\n");
        }

        Uart::<UART0_BASE>::emergency_write_str("=============================\n\n");
    }

    loop {
        core::hint::spin_loop();
    }
}
