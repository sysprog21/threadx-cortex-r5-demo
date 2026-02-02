//! Shared code for the ThreadX and Rust demonstration on the Cortex-R5.

#![no_std]

pub mod buffered_uart;
pub mod config;
pub mod exception;
pub mod pl011_uart;
pub mod pl190_vic;
pub mod sp804_timer;
pub mod startup;
