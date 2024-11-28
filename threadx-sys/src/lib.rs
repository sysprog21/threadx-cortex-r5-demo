//! Auto-generated bindings for ThreadX 6.4.1
//!
//! ```quote
//!     /**************************************************************************/
//!     /* Copyright (c) 2024 Microsoft Corporation                               */
//!     /*                                                                        */
//!     /* This program and the accompanying materials are made available under   */
//!     /* the terms of the MIT License which is available at                     */
//!     /*  https://opensource.org/licenses/MIT.                                  */
//!     /*                                                                        */
//!     /* SPDX-License-Identifier: MIT                                           */
//!     /**************************************************************************/
//! ```

// The notice above applies to the build output of this crate, including the
// bindings.rs file.
//
// The notice below applies to this file, without the bindings.rs file.
//
#![no_std]
#![allow(non_snake_case)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]
#![allow(clippy::missing_safety_doc)]

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

// Bindgen couldn't pick these constants out of the header file
// So I added them manually.

pub const TX_NO_WAIT: ULONG = 0;
pub const TX_WAIT_FOREVER: ULONG = 0xFFFFFFFF;
pub const TX_AND: UINT = 2;
pub const TX_AND_CLEAR: UINT = 3;
pub const TX_OR: UINT = 0;
pub const TX_OR_CLEAR: UINT = 1;
pub const TX_1_ULONG: UINT = 1;
pub const TX_2_ULONG: UINT = 2;
pub const TX_4_ULONG: UINT = 4;
pub const TX_8_ULONG: UINT = 8;
pub const TX_16_ULONG: UINT = 16;
pub const TX_NO_TIME_SLICE: ULONG = 0;
pub const TX_AUTO_START: UINT = 1;
pub const TX_DONT_START: UINT = 0;
pub const TX_AUTO_ACTIVATE: UINT = 1;
pub const TX_NO_ACTIVATE: UINT = 0;
pub const TX_TRUE: UINT = 1;
pub const TX_FALSE: UINT = 0;
pub const TX_INHERIT: UINT = 1;
pub const TX_NO_INHERIT: UINT = 0;
pub const TX_THREAD_ENTRY: UINT = 0;
pub const TX_THREAD_EXIT: UINT = 1;
pub const TX_NO_SUSPENSIONS: UINT = 0;
pub const TX_NO_MESSAGES: UINT = 0;
pub const TX_EMPTY: ULONG = 0;
pub const TX_CLEAR_ID: ULONG = 0;

/// Operation completed successfully
pub const TX_SUCCESS: UINT = 0;
