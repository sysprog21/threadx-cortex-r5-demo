//! Implementation for the Cadence Triple Timer Counter (TTC) used on Zynq UltraScale+.
//!
//! Each TTC module contains 3 independent timer channels. The TTC provides:
//! - 16-bit prescaler
//! - 16-bit counter (can count up or down)
//! - Interval and overflow modes
//! - 3 match registers per channel

#[cfg(feature = "zynqmp")]
use crate::config_zynqmp::{ttc, TTC0_BASE};

/// Timer channel within a TTC module.
#[derive(Clone, Copy)]
pub enum Channel {
    Channel0 = 0,
    Channel1 = 1,
    Channel2 = 2,
}

/// Supported timer modes.
pub enum Mode {
    /// Overflow mode: counter counts from 0 to 0xFFFF and wraps
    Overflow,
    /// Interval mode: counter counts from 0 to interval value and resets
    Interval,
}

/// TTC0 on Zynq UltraScale+.
#[cfg(feature = "zynqmp")]
pub type Ttc0 = Ttc<TTC0_BASE>;

/// Device driver for a Cadence Triple Timer Counter.
pub struct Ttc<const ADDR: usize> {
    channel: Channel,
}

#[cfg(feature = "zynqmp")]
impl Ttc<TTC0_BASE> {
    /// Create a new TTC object for TTC0 channel 0
    ///
    /// # Safety
    /// Only construct one object per TTC channel at any given time.
    pub unsafe fn new_ttc0(channel: Channel) -> Self {
        Ttc { channel }
    }
}

impl<const ADDR: usize> Ttc<ADDR> {
    /// Offset between channels (each channel has its own register set).
    const CHANNEL_OFFSET: usize = 4;

    /// Read a register for this channel.
    fn read_reg(&self, offset: usize) -> u32 {
        #[cfg(feature = "zynqmp")]
        unsafe {
            // Register layout: base + offset + (channel * 4)
            let ptr =
                (ADDR + offset + (self.channel as usize) * Self::CHANNEL_OFFSET) as *const u32;
            ptr.read_volatile()
        }
        #[cfg(not(feature = "zynqmp"))]
        {
            let _ = offset; // Silence unused warning
            0
        }
    }

    /// Write a register for this channel.
    fn write_reg(&self, offset: usize, value: u32) {
        #[cfg(feature = "zynqmp")]
        unsafe {
            let ptr = (ADDR + offset + (self.channel as usize) * Self::CHANNEL_OFFSET) as *mut u32;
            ptr.write_volatile(value);
        }
    }

    /// Initialize the timer with the given interval and mode.
    ///
    /// # Arguments
    /// * `interval` - The interval value (only used in Interval mode)
    /// * `mode` - Timer mode (Overflow or Interval)
    /// * `enable_interrupt` - Whether to enable the interval/overflow interrupt
    pub fn init(&mut self, interval: u16, mode: Mode, enable_interrupt: bool) {
        #[cfg(feature = "zynqmp")]
        {
            // Disable counter first
            self.write_reg(ttc::CNT_CTRL_OFFSET, ttc::CNT_CTRL_DIS);

            // NOTE: Do NOT reset CLK_CTRL here - prescaler may have been configured
            // by set_prescaler() before init() was called.

            // Set interval value
            self.write_reg(ttc::INTERVAL_OFFSET, interval as u32);

            // Reset counter but keep disabled; caller must call start().
            let interval_bit = match mode {
                Mode::Interval => ttc::CNT_CTRL_INT,
                Mode::Overflow => 0,
            };
            self.write_reg(
                ttc::CNT_CTRL_OFFSET,
                ttc::CNT_CTRL_RST | ttc::CNT_CTRL_DIS | interval_bit,
            );

            // Configure interrupts
            if enable_interrupt {
                let int_mask = match mode {
                    Mode::Interval => ttc::INT_INTERVAL,
                    Mode::Overflow => ttc::INT_OVERFLOW,
                };
                self.write_reg(ttc::IER_OFFSET, int_mask);
            }
        }
    }

    /// Set the prescaler value.
    ///
    /// The prescaler divides the input clock by 2^(prescale+1).
    /// Valid values are 0-15.
    pub fn set_prescaler(&mut self, prescale: u8) {
        #[cfg(feature = "zynqmp")]
        {
            let prescale = prescale.min(15);
            let value = ttc::CLK_CTRL_PS_EN | ((prescale as u32) << ttc::CLK_CTRL_PS_SHIFT);
            self.write_reg(ttc::CLK_CTRL_OFFSET, value);
        }
    }

    /// Start the timer.
    pub fn start(&mut self) {
        #[cfg(feature = "zynqmp")]
        {
            let ctrl = self.read_reg(ttc::CNT_CTRL_OFFSET);
            self.write_reg(ttc::CNT_CTRL_OFFSET, ctrl & !ttc::CNT_CTRL_DIS);
        }
    }

    /// Stop the timer.
    pub fn stop(&mut self) {
        #[cfg(feature = "zynqmp")]
        {
            let ctrl = self.read_reg(ttc::CNT_CTRL_OFFSET);
            self.write_reg(ttc::CNT_CTRL_OFFSET, ctrl | ttc::CNT_CTRL_DIS);
        }
    }

    /// Read the current counter value.
    pub fn read_counter(&self) -> u32 {
        self.read_reg(ttc::CNT_VALUE_OFFSET)
    }

    /// Clear interrupt for a specific channel without requiring an instance.
    ///
    /// Intended for ISR use where constructing a `Ttc` would violate the
    /// single-owner safety contract (the owner lives in `tx_application_define`).
    ///
    /// Returns the ISR value (read-to-clear).
    #[inline]
    pub fn clear_interrupt_static(channel: Channel) -> u32 {
        #[cfg(feature = "zynqmp")]
        unsafe {
            let ptr =
                (ADDR + ttc::ISR_OFFSET + (channel as usize) * Self::CHANNEL_OFFSET) as *const u32;
            ptr.read_volatile()
        }
        #[cfg(not(feature = "zynqmp"))]
        {
            let _ = channel;
            0
        }
    }

    /// Read and clear pending interrupt status.
    ///
    /// On Cadence TTC, reading the ISR register clears the pending bits
    /// (read-to-clear). This means every read is destructive -- there is
    /// no way to check status without clearing it.
    ///
    /// Returns the ISR value at the time of the read. Test individual bits
    /// against `ttc::INT_INTERVAL`, `ttc::INT_OVERFLOW`, etc.
    pub fn clear_interrupt(&self) -> u32 {
        self.read_reg(ttc::ISR_OFFSET)
    }
}
