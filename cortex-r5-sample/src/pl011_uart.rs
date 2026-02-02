//! Implementation for the Arm PL011 UART

use crate::config::{uart, UART0_BASE};

/// Device driver for a virtual PL011 UART
///
/// While some essential initialization steps may be omitted, it functions
/// correctly on QEMU
pub struct Uart<const ADDR: usize>();

impl Uart<UART0_BASE> {
    /// Create a new UART object for UART0
    ///
    /// # Safety
    /// Only construct one object per UART at any given time.
    pub unsafe fn new_uart0() -> Self {
        let mut u = Uart();
        u.set_control(uart::CONTROL_UARTEN | uart::CONTROL_TXE);
        u
    }
}

impl<const ADDR: usize> Uart<ADDR> {
    const FLAG_TXFF: u32 = uart::FLAG_TXFF;

    const DATA_OFFSET: usize = uart::DATA_OFFSET;
    const FLAG_OFFSET: usize = uart::FLAG_OFFSET;
    const CONTROL_OFFSET: usize = uart::CONTROL_OFFSET;

    /// Emergency write function that bypasses all locks and state checks.
    ///
    /// # Safety
    /// Safe to call from fault handlers, stack error handlers, and interrupt context.
    /// Does NOT use any mutexes or shared state. Writes directly to hardware registers.
    ///
    /// ## Preconditions (Caller Responsibility)
    /// - UART must be initialized (control register configured via `new_uart0()` or equivalent)
    /// - UART clock must be enabled by hardware initialization code
    /// - UART MMIO region must be mapped in MPU as Device or Strongly-Ordered memory
    ///   (not Normal/Cacheable) to ensure volatile semantics work correctly
    /// - On production systems with D-cache, ensure UART address range is non-cacheable
    ///
    /// ## Robustness Features
    /// - Bounded wait: Times out and drops characters if UART stalls (prevents deadlock)
    /// - Memory barriers: Uses `dsb sy` to ensure MMIO writes complete before continuing
    /// - No stack allocation: Suitable for stack overflow handlers
    ///
    /// ## Use Cases
    /// - Stack overflow handlers (where stack is exhausted)
    /// - Panic handlers (where locks may be held)
    /// - Fault exception handlers (before system halt)
    pub unsafe fn emergency_write_str(s: &str) {
        const EMERGENCY_UART_TIMEOUT: u32 = 10000;

        let base = ADDR as *mut u32;
        let flag_ptr = base.add(Self::FLAG_OFFSET);
        let data_ptr = base.add(Self::DATA_OFFSET);

        for b in s.bytes() {
            // Bounded wait for TXFF (TX FIFO Full) to clear
            // If UART is misconfigured or clocks disabled, this prevents infinite hang
            let mut timeout = EMERGENCY_UART_TIMEOUT;
            while (flag_ptr.read_volatile() & Self::FLAG_TXFF) != 0 {
                timeout = timeout.saturating_sub(1);
                if timeout == 0 {
                    // UART stuck - drop this character and continue
                    // Better to lose chars than deadlock diagnostics
                    break;
                }
            }

            // Only write if FIFO has space (or we timed out and are dropping)
            if timeout > 0 {
                // Compiler fence: prevent reordering of this write with surrounding code
                core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

                // Write data to UART
                data_ptr.write_volatile(b as u32);

                // Hardware memory barrier: ensure MMIO write completes before continuing
                // Critical for correctness if D-cache enabled or store buffer active
                core::arch::asm!("dsb sy", options(nostack, preserves_flags));
            }
        }
    }

    /// Write a byte (blocking if there is no space)
    pub fn write(&mut self, byte: u8) {
        // Check the TX FIFO Full bit
        while (self.get_flags() & Self::FLAG_TXFF) != 0 {}
        self.write_data(byte);
    }

    /// Write to the data register
    fn write_data(&mut self, value: u8) {
        unsafe {
            let ptr = (ADDR as *mut u32).add(Self::DATA_OFFSET);
            ptr.write_volatile(value as u32);
        }
    }

    /// Read from the Flag Register
    fn get_flags(&mut self) -> u32 {
        unsafe {
            let ptr = (ADDR as *const u32).add(Self::FLAG_OFFSET);
            ptr.read_volatile()
        }
    }

    /// Write to the control register
    fn set_control(&mut self, value: u32) {
        unsafe {
            let ptr = (ADDR as *mut u32).add(Self::CONTROL_OFFSET);
            ptr.write_volatile(value);
        }
    }
}

impl<const N: usize> core::fmt::Write for Uart<N> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for b in s.bytes() {
            self.write(b);
        }
        Ok(())
    }
}
