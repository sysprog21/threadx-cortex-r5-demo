//! Implementation for the Cadence (Xilinx) UART used on Zynq UltraScale+.
//!
//! The Cadence UART has a different register layout than the ARM PL011.
//! Key differences:
//! - TX/RX share a single FIFO register
//! - Channel status register for FIFO status (not Flag register)
//! - Separate control and mode registers

#[cfg(feature = "zynqmp")]
use crate::config_zynqmp::{uart, UART0_BASE};

/// Device driver for a Cadence UART
///
/// Used on Zynq UltraScale+ and other Xilinx devices.
pub struct CadenceUart<const ADDR: usize>();

#[cfg(feature = "zynqmp")]
impl CadenceUart<UART0_BASE> {
    /// Create a new UART object for UART0
    ///
    /// # Safety
    /// Only construct one object per UART at any given time.
    pub unsafe fn new_uart0() -> Self {
        let mut u = CadenceUart();
        u.init();
        u
    }
}

impl<const ADDR: usize> CadenceUart<ADDR> {
    /// Initialize the UART for basic TX operation.
    ///
    /// On QEMU, the UART is typically already configured by the emulator,
    /// but we still need to enable TX.
    fn init(&mut self) {
        #[cfg(feature = "zynqmp")]
        unsafe {
            let base = ADDR as *mut u32;

            // Reset TX and RX paths
            base.byte_add(uart::CONTROL_OFFSET)
                .write_volatile(uart::CTRL_TXRES | uart::CTRL_RXRES);

            // Configure mode: 8N1, normal mode, use reference clock
            base.byte_add(uart::MODE_OFFSET).write_volatile(
                uart::MODE_CHMODE_NORMAL
                    | uart::MODE_NBSTOP_1
                    | uart::MODE_PAR_NONE
                    | uart::MODE_CHRL_8
                    | uart::MODE_CLKS_REF,
            );

            // Disable all interrupts initially
            base.byte_add(uart::IDR_OFFSET).write_volatile(0xFFFF_FFFF);

            // Enable TX (and RX for completeness)
            base.byte_add(uart::CONTROL_OFFSET)
                .write_volatile(uart::CTRL_TXEN | uart::CTRL_RXEN);
        }
    }

    /// Read the channel status register.
    fn get_status(&self) -> u32 {
        #[cfg(feature = "zynqmp")]
        unsafe {
            let base = ADDR as *const u32;
            base.byte_add(uart::CHANNEL_STS_OFFSET).read_volatile()
        }
        #[cfg(not(feature = "zynqmp"))]
        0
    }

    /// Check if TX FIFO is full.
    fn is_tx_full(&self) -> bool {
        #[cfg(feature = "zynqmp")]
        {
            (self.get_status() & uart::STS_TXFULL) != 0
        }
        #[cfg(not(feature = "zynqmp"))]
        false
    }

    /// Write a byte to the TX FIFO.
    fn write_fifo(&mut self, byte: u8) {
        #[cfg(feature = "zynqmp")]
        unsafe {
            let base = ADDR as *mut u32;
            base.byte_add(uart::FIFO_OFFSET).write_volatile(byte as u32);
        }
    }

    /// Write a byte (blocking if TX FIFO is full).
    pub fn write(&mut self, byte: u8) {
        while self.is_tx_full() {}
        self.write_fifo(byte);
    }

    /// Emergency write function that bypasses all locks and state checks.
    ///
    /// # Safety
    /// Safe to call from fault handlers, stack error handlers, and interrupt context.
    /// Does NOT use any mutexes or shared state. Writes directly to hardware registers.
    ///
    /// ## Preconditions (Caller Responsibility)
    /// - UART must be initialized (control register configured)
    /// - UART clock must be enabled
    /// - UART MMIO region must be mapped as Device memory
    ///
    /// ## Robustness Features
    /// - Bounded wait: Times out and drops characters if UART stalls
    /// - Memory barriers: Uses `dsb sy` to ensure MMIO writes complete
    /// - No stack allocation: Suitable for stack overflow handlers
    #[cfg(feature = "zynqmp")]
    pub unsafe fn emergency_write_str(s: &str) {
        const EMERGENCY_UART_TIMEOUT: u32 = 10000;

        let base = ADDR as *mut u32;
        let status_ptr = base.byte_add(uart::CHANNEL_STS_OFFSET) as *const u32;
        let fifo_ptr = base.byte_add(uart::FIFO_OFFSET);

        for b in s.bytes() {
            // Bounded wait for TX FIFO to have space
            let mut timeout = EMERGENCY_UART_TIMEOUT;
            while (status_ptr.read_volatile() & uart::STS_TXFULL) != 0 {
                timeout = timeout.saturating_sub(1);
                if timeout == 0 {
                    break;
                }
            }

            if timeout > 0 {
                core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
                fifo_ptr.write_volatile(b as u32);
                core::arch::asm!("dsb sy", options(nostack, preserves_flags));
            }
        }
    }

    /// Provide a no-op emergency_write_str when zynqmp feature is disabled.
    #[cfg(not(feature = "zynqmp"))]
    pub unsafe fn emergency_write_str(_s: &str) {}
}

impl<const N: usize> ufmt::uWrite for CadenceUart<N> {
    type Error = core::convert::Infallible;

    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        for b in s.bytes() {
            self.write(b);
        }
        Ok(())
    }
}

/// Global UART writer for use with println_uart! macro.
///
/// Protected by critical section to prevent aliasing UB.
#[cfg(feature = "zynqmp")]
#[doc(hidden)] // Used by println_uart_zynqmp! macro
pub static GLOBAL_UART: critical_section::Mutex<
    core::cell::RefCell<Option<CadenceUart<UART0_BASE>>>,
> = critical_section::Mutex::new(core::cell::RefCell::new(None));

/// Initialize the global UART writer.
///
/// # Safety
/// Must be called exactly once, before any println_uart! usage.
/// UART hardware must be initialized (clocks enabled, pins muxed).
#[cfg(feature = "zynqmp")]
pub unsafe fn init_global_uart() {
    critical_section::with(|cs| {
        *GLOBAL_UART.borrow_ref_mut(cs) = Some(CadenceUart::new_uart0());
    });
}

/// Print to UART0 (Zynq UltraScale+ version).
///
/// Uses the global UART writer. Must call `init_global_uart()` first.
#[cfg(feature = "zynqmp")]
#[macro_export]
macro_rules! println_uart_zynqmp {
    ($($arg:tt)*) => {{
        use ufmt::uwriteln;
        critical_section::with(|cs| {
            if let Some(ref mut uart) = *$crate::cadence_uart::GLOBAL_UART.borrow_ref_mut(cs) {
                let _ = uwriteln!(uart, $($arg)*);
            }
        });
    }};
}
