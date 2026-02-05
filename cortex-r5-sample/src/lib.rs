//! Shared code for the ThreadX and Rust demonstration on the Cortex-R5.
//!
//! When the `host-test` feature is enabled, only the `safe` and `threadx_error` modules
//! are exposed. This allows compile-fail tests to run on host platforms without ARM dependencies.

#![cfg_attr(not(feature = "host-test"), no_std)]

// Hardware-specific modules only available when NOT in host-test mode
#[cfg(not(feature = "host-test"))]
pub mod buffered_uart;
#[cfg(not(feature = "host-test"))]
pub mod config;
#[cfg(not(feature = "host-test"))]
pub mod exception;
#[cfg(not(feature = "host-test"))]
pub mod pl011_uart;
#[cfg(not(feature = "host-test"))]
pub mod pl190_vic;
#[cfg(not(feature = "host-test"))]
pub mod sp804_timer;
#[cfg(not(feature = "host-test"))]
pub mod startup;
#[cfg(not(feature = "host-test"))]
pub mod thread;

// Modules available in both host-test and embedded modes
pub mod safe;
pub mod spsc;
pub mod threadx_error;
