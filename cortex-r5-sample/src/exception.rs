//! ARM Cortex-R5 Exception Handlers
//!
//! Provides diagnostic fault handlers for hardware exceptions that occur outside
//! Rust's panic machinery (misaligned access, invalid instructions, MMU violations).

// Board-specific imports (zynqmp takes precedence if both are enabled)
#[cfg(all(feature = "versatileab", not(feature = "zynqmp")))]
use crate::config::UART0_BASE;
#[cfg(feature = "zynqmp")]
use crate::config_zynqmp::UART0_BASE;

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

/// Check if a pointer falls within a valid RAM region.
///
/// Used by fault handlers to avoid dereferencing MMIO or unmapped addresses
/// during crash diagnostics, which would trigger a recursive abort.
#[inline]
fn is_valid_ram_range(addr: usize, len: usize) -> bool {
    if len == 0 {
        return false;
    }
    let Some(end) = addr.checked_add(len - 1) else {
        return false;
    };

    #[cfg(all(feature = "versatileab", not(feature = "zynqmp")))]
    {
        // QEMU VersatileAB: 16MB flat RAM at 0x0
        end < 0x0100_0000
    }
    #[cfg(feature = "zynqmp")]
    {
        use crate::config_zynqmp::*;
        // DDR:  0x0000_0000 .. 0x7FFF_FFFF (2GB)
        // ATCM: 0xFFE0_0000 .. 0xFFE0_FFFF (64K)
        // BTCM: 0xFFE2_0000 .. 0xFFE2_FFFF (64K)
        // OCM:  0xFFFC_0000 .. 0xFFFF_FFFF (256K)
        (end < DDR_BASE + DDR_SIZE)
            || (addr >= ATCM_BASE && end < ATCM_BASE + ATCM_SIZE)
            || (addr >= BTCM_BASE && end < BTCM_BASE + BTCM_SIZE)
            || (addr >= OCM_BASE && (end - OCM_BASE) < OCM_SIZE)
    }
}

unsafe fn write_thread_name(thread: *mut threadx_sys::TX_THREAD) {
    const MAX_THREAD_NAME_LEN: usize = 32;

    let thread_valid = !thread.is_null()
        && is_valid_ram_range(
            thread as usize,
            core::mem::size_of::<threadx_sys::TX_THREAD>(),
        )
        && (thread as usize & (core::mem::align_of::<threadx_sys::TX_THREAD>() - 1)) == 0;

    if !thread_valid {
        emergency_write("Thread: <none/invalid>\n");
        return;
    }

    let name_ptr = (*thread).tx_thread_name;
    if name_ptr.is_null() || !is_valid_ram_range(name_ptr as usize, MAX_THREAD_NAME_LEN) {
        emergency_write("Thread: <invalid-name>\n");
        return;
    }

    let name_len = c_strlen_safe(name_ptr as *const u8, MAX_THREAD_NAME_LEN);
    let name_bytes = core::slice::from_raw_parts(name_ptr as *const u8, name_len);
    if let Ok(s) = core::str::from_utf8(name_bytes) {
        emergency_write("Thread: ");
        emergency_write(s);
        emergency_write("\n");
    } else {
        emergency_write("Thread: <invalid-utf8>\n");
    }
}

/// Board-agnostic emergency UART write function.
///
/// # Safety
/// Safe to call from fault handlers - uses direct MMIO, no locks.
#[inline(always)]
unsafe fn emergency_write(s: &str) {
    #[cfg(all(feature = "versatileab", not(feature = "zynqmp")))]
    {
        use crate::pl011_uart::Uart;
        Uart::<UART0_BASE>::emergency_write_str(s);
    }
    #[cfg(feature = "zynqmp")]
    {
        use crate::cadence_uart::CadenceUart;
        CadenceUart::<UART0_BASE>::emergency_write_str(s);
    }
}

/// Halt the system via semihosting exit.
///
/// Uses the same mechanism as the panic handler to cleanly terminate QEMU
/// instead of spinning forever, which would hang CI pipelines.
fn halt_system() -> ! {
    const SYS_REPORTEXC: u32 = 0x18;
    // ADP_Stopped_InternalError (0x20024) signals abnormal exit
    const ADP_STOPPED_INTERNAL_ERROR: u32 = 0x20024;
    loop {
        unsafe {
            core::arch::asm!(
                "svc 0x123456",
                in("r0") SYS_REPORTEXC,
                in("r1") ADP_STOPPED_INTERNAL_ERROR,
            );
        }
    }
}

/// Format a u32 as 8-digit uppercase hex into buf after prefix, followed by newline.
///
/// No core::fmt dependency -- pure byte manipulation.
fn format_hex<'a>(buf: &'a mut [u8], prefix: &str, value: u32) -> &'a str {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    let pb = prefix.as_bytes();
    let need = pb.len() + 8 + 1; // prefix + 8 hex digits + newline
    if buf.len() < need {
        return "";
    }
    buf[..pb.len()].copy_from_slice(pb);
    let off = pb.len();
    for i in 0..8 {
        buf[off + i] = HEX[((value >> (28 - i * 4)) & 0xF) as usize];
    }
    buf[off + 8] = b'\n';
    // Safety: prefix is valid UTF-8, hex digits and newline are ASCII
    unsafe { core::str::from_utf8_unchecked(&buf[..need]) }
}

/// Read a CP15 coprocessor register.
macro_rules! read_cp15 {
    ($crn:literal, $crm:literal, $op2:literal) => {{
        let value: u32;
        unsafe {
            core::arch::asm!(
                concat!("mrc p15, 0, {}, ", $crn, ", ", $crm, ", ", $op2),
                out(reg) value,
                options(nostack, nomem)
            );
        }
        value
    }};
}

/// Data Fault Address Register (CP15 c6, c0, 0)
#[inline(always)]
fn read_dfar() -> u32 {
    read_cp15!("c6", "c0", "0")
}

/// Data Fault Status Register (CP15 c5, c0, 0)
#[inline(always)]
fn read_dfsr() -> u32 {
    read_cp15!("c5", "c0", "0")
}

/// Instruction Fault Address Register (CP15 c6, c0, 2)
#[inline(always)]
fn read_ifar() -> u32 {
    read_cp15!("c6", "c0", "2")
}

/// Instruction Fault Status Register (CP15 c5, c0, 1)
#[inline(always)]
fn read_ifsr() -> u32 {
    read_cp15!("c5", "c0", "1")
}

/// Decode ARM fault status register (combines bits [3:0] with bit 10).
#[inline]
fn decode_fault_status(fsr: u32) -> u32 {
    (fsr & 0xF) | ((fsr & 0x400) >> 6)
}

/// Emit a fault report via emergency UART and halt.
///
/// Prints the banner, PC, optional fault registers, current thread name,
/// and terminates via semihosting.
unsafe fn report_fault(banner: &str, trailer: &str, pc: u32, regs: &[(&str, u32)]) -> ! {
    let thread = threadx_sys::_tx_thread_identify();

    emergency_write(banner);

    let mut buf = [0u8; 64];
    emergency_write(format_hex(&mut buf, "PC:            0x", pc));
    for &(label, value) in regs {
        emergency_write(format_hex(&mut buf, label, value));
    }

    write_thread_name(thread);
    emergency_write(trailer);

    halt_system();
}

/// Data Abort Exception Handler
///
/// Called when a data access causes an abort (misaligned access, access to
/// invalid memory region, MPU/MMU violation).
///
/// # Parameters
/// - `pc`: Faulting instruction address (LR - 8, adjusted by assembly)
#[no_mangle]
pub extern "C" fn data_abort_handler(pc: u32) -> ! {
    let dfar = read_dfar();
    let dfsr = read_dfsr();
    let status = decode_fault_status(dfsr);

    unsafe {
        report_fault(
            "\n=== DATA ABORT ===\n",
            "==================\n\n",
            pc,
            &[
                ("Fault Address: 0x", dfar),
                ("Fault Status:  0x", dfsr),
                ("Fault Type:    0x", status),
            ],
        )
    }
}

/// Prefetch Abort Exception Handler
///
/// Called when instruction fetch causes an abort (branch to invalid address,
/// attempt to execute data region, MPU/MMU violation on instruction fetch).
///
/// # Parameters
/// - `pc`: Faulting instruction address (LR - 4, adjusted by assembly)
#[no_mangle]
pub extern "C" fn prefetch_abort_handler(pc: u32) -> ! {
    let ifar = read_ifar();
    let ifsr = read_ifsr();
    let status = decode_fault_status(ifsr);

    unsafe {
        report_fault(
            "\n=== PREFETCH ABORT ===\n",
            "======================\n\n",
            pc,
            &[
                ("Fault Address: 0x", ifar),
                ("Fault Status:  0x", ifsr),
                ("Fault Type:    0x", status),
            ],
        )
    }
}

/// Undefined Instruction Exception Handler
///
/// Called when the processor encounters an instruction it cannot decode
/// (corrupted code, invalid opcode, FPU instruction without FPU enabled).
///
/// # Parameters
/// - `pc`: Faulting instruction address (LR - 4, adjusted by assembly)
#[no_mangle]
pub extern "C" fn undefined_handler(pc: u32) -> ! {
    unsafe {
        report_fault(
            "\n=== UNDEFINED INSTRUCTION ===\n",
            "=============================\n\n",
            pc,
            &[],
        )
    }
}
