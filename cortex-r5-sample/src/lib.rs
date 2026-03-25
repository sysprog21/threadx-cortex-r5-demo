//! Shared code for the ThreadX and Rust demonstration on the Cortex-R5.
//!
//! When the `host-test` feature is enabled, only the `safe` and `threadx_error` modules
//! are exposed. This allows compile-fail tests to run on host platforms without ARM dependencies.
//!
//! # Board Support
//!
//! The crate supports multiple boards via Cargo features:
//! - `versatileab` (default): QEMU VersatileAB with PL011 UART, SP804 Timer, PL190 VIC
//! - `zynqmp`: Zynq UltraScale+ RPU with Cadence UART, TTC Timer, GIC

#![cfg_attr(not(feature = "host-test"), no_std)]

// Board features are mutually exclusive
#[cfg(all(feature = "versatileab", feature = "zynqmp"))]
compile_error!(
    "Features `versatileab` and `zynqmp` are mutually exclusive. \
     Use: cargo build --no-default-features --features embedded,zynqmp"
);

#[cfg(all(
    not(feature = "host-test"),
    not(feature = "versatileab"),
    not(feature = "zynqmp")
))]
compile_error!("Select one board feature: `versatileab` or `zynqmp`.");

// Hardware-specific modules only available when NOT in host-test mode
#[cfg(not(feature = "host-test"))]
pub mod buffered_uart;
#[cfg(not(feature = "host-test"))]
pub mod exception;
#[cfg(not(feature = "host-test"))]
pub mod startup;
#[cfg(not(feature = "host-test"))]
pub mod thread;

// Board-specific configuration modules
#[cfg(all(not(feature = "host-test"), feature = "versatileab"))]
pub mod config;
#[cfg(all(not(feature = "host-test"), feature = "zynqmp"))]
pub mod config_zynqmp;

// VersatileAB board drivers (default)
#[cfg(all(not(feature = "host-test"), feature = "versatileab"))]
pub mod pl011_uart;
#[cfg(all(not(feature = "host-test"), feature = "versatileab"))]
pub mod pl190_vic;
#[cfg(all(not(feature = "host-test"), feature = "versatileab"))]
pub mod sp804_timer;

// Zynq UltraScale+ board drivers
#[cfg(all(not(feature = "host-test"), feature = "zynqmp"))]
pub mod cadence_ttc;
#[cfg(all(not(feature = "host-test"), feature = "zynqmp"))]
pub mod cadence_uart;
#[cfg(all(not(feature = "host-test"), feature = "zynqmp"))]
pub mod gic;

// Modules available in both host-test and embedded modes
pub mod safe;
pub mod spsc;
pub mod threadx_error;
