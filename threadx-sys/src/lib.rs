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
pub const TX_SUCCESS: UINT = 0x00;

// Thread states
pub const TX_READY: UINT = 0;
pub const TX_COMPLETED: UINT = 1;
pub const TX_TERMINATED: UINT = 2;
pub const TX_SUSPENDED: UINT = 3;
pub const TX_SLEEP: UINT = 4;
pub const TX_QUEUE_SUSP: UINT = 5;
pub const TX_SEMAPHORE_SUSP: UINT = 6;
pub const TX_EVENT_FLAG: UINT = 7;
pub const TX_BLOCK_MEMORY: UINT = 8;
pub const TX_BYTE_MEMORY: UINT = 9;
pub const TX_IO_DRIVER: UINT = 10;
pub const TX_FILE: UINT = 11;
pub const TX_TCP_IP: UINT = 12;
pub const TX_MUTEX_SUSP: UINT = 13;
pub const TX_PRIORITY_CHANGE: UINT = 14;

// Error codes
pub const TX_DELETED: UINT = 0x01;
pub const TX_POOL_ERROR: UINT = 0x02;
pub const TX_PTR_ERROR: UINT = 0x03;
pub const TX_WAIT_ERROR: UINT = 0x04;
pub const TX_SIZE_ERROR: UINT = 0x05;
pub const TX_GROUP_ERROR: UINT = 0x06;
pub const TX_NO_EVENTS: UINT = 0x07;
pub const TX_OPTION_ERROR: UINT = 0x08;
pub const TX_QUEUE_ERROR: UINT = 0x09;
pub const TX_QUEUE_EMPTY: UINT = 0x0A;
pub const TX_QUEUE_FULL: UINT = 0x0B;
pub const TX_SEMAPHORE_ERROR: UINT = 0x0C;
pub const TX_NO_INSTANCE: UINT = 0x0D;
pub const TX_THREAD_ERROR: UINT = 0x0E;
pub const TX_PRIORITY_ERROR: UINT = 0x0F;
pub const TX_NO_MEMORY: UINT = 0x10;
pub const TX_START_ERROR: UINT = 0x10;
pub const TX_DELETE_ERROR: UINT = 0x11;
pub const TX_RESUME_ERROR: UINT = 0x12;
pub const TX_CALLER_ERROR: UINT = 0x13;
pub const TX_SUSPEND_ERROR: UINT = 0x14;
pub const TX_TIMER_ERROR: UINT = 0x15;
pub const TX_TICK_ERROR: UINT = 0x16;
pub const TX_ACTIVATE_ERROR: UINT = 0x17;
pub const TX_THRESH_ERROR: UINT = 0x18;
pub const TX_SUSPEND_LIFTED: UINT = 0x19;
pub const TX_WAIT_ABORTED: UINT = 0x1A;
pub const TX_WAIT_ABORT_ERROR: UINT = 0x1B;
pub const TX_MUTEX_ERROR: UINT = 0x1C;
pub const TX_NOT_AVAILABLE: UINT = 0x1D;
pub const TX_NOT_OWNED: UINT = 0x1E;
pub const TX_INHERIT_ERROR: UINT = 0x1F;
pub const TX_NOT_DONE: UINT = 0x20;
pub const TX_CEILING_EXCEEDED: UINT = 0x21;
pub const TX_INVALID_CEILING: UINT = 0x22;
pub const TX_FEATURE_NOT_ENABLED: UINT = 0xFF;

// Note: TX_INT_DISABLE and TX_INT_ENABLE are defined in bindings.rs
