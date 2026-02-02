//! Stub types for compile-fail testing on host platforms.
//!
//! This module provides minimal type definitions that mirror threadx-sys types,
//! allowing the safe module's Rust type structure to be validated on host
//! platforms without requiring actual ThreadX bindings.
//!
//! These stubs are only compiled when the `host-test` feature is enabled.

#![allow(non_camel_case_types)]
#![allow(dead_code)]

/// Stub for ThreadX unsigned int type
pub type UINT = u32;

/// Stub for ThreadX unsigned long type
pub type ULONG = u32;

/// Stub for ThreadX char type
pub type CHAR = i8;

/// Stub TX_SUCCESS constant
pub const TX_SUCCESS: UINT = 0;

/// Stub TX_NO_TIME_SLICE constant
pub const TX_NO_TIME_SLICE: ULONG = 0;

/// Stub TX_AUTO_START constant
pub const TX_AUTO_START: UINT = 1;

/// Stub TX_DONT_START constant
pub const TX_DONT_START: UINT = 0;

/// Stub TX_INHERIT constant
pub const TX_INHERIT: UINT = 1;

/// Stub TX_NO_INHERIT constant
pub const TX_NO_INHERIT: UINT = 0;

/// Stub TX_WAIT_FOREVER constant
pub const TX_WAIT_FOREVER: ULONG = 0xFFFF_FFFF;

/// Stub TX_NO_WAIT constant
pub const TX_NO_WAIT: ULONG = 0;

/// Stub TX_NOT_AVAILABLE constant
pub const TX_NOT_AVAILABLE: UINT = 1;

/// Stub TX_WAIT_ABORTED constant
pub const TX_WAIT_ABORTED: UINT = 2;

/// Stub TX_MUTEX_ERROR constant
pub const TX_MUTEX_ERROR: UINT = 3;

/// Stub TX_CALLER_ERROR constant
pub const TX_CALLER_ERROR: UINT = 4;

/// Stub TX_QUEUE_FULL constant
pub const TX_QUEUE_FULL: UINT = 5;

/// Stub TX_QUEUE_EMPTY constant
pub const TX_QUEUE_EMPTY: UINT = 6;

/// Stub TX_QUEUE_ERROR constant
pub const TX_QUEUE_ERROR: UINT = 7;

/// Stub for TX_THREAD structure (opaque, size doesn't matter for type checking)
#[repr(C)]
pub struct TX_THREAD {
    _opaque: [u8; 256],
    pub tx_thread_name: *mut CHAR,
    pub tx_thread_priority: UINT,
    pub tx_thread_state: UINT,
}

impl Default for TX_THREAD {
    fn default() -> Self {
        Self {
            _opaque: [0; 256],
            tx_thread_name: core::ptr::null_mut(),
            tx_thread_priority: 0,
            tx_thread_state: 0,
        }
    }
}

/// Stub for TX_MUTEX structure
#[repr(C)]
pub struct TX_MUTEX {
    _opaque: [u8; 128],
    pub tx_mutex_name: *mut CHAR,
    pub tx_mutex_owner: *mut TX_THREAD,
    pub tx_mutex_ownership_count: ULONG,
}

impl Default for TX_MUTEX {
    fn default() -> Self {
        Self {
            _opaque: [0; 128],
            tx_mutex_name: core::ptr::null_mut(),
            tx_mutex_owner: core::ptr::null_mut(),
            tx_mutex_ownership_count: 0,
        }
    }
}

/// Stub for TX_QUEUE structure
#[repr(C)]
pub struct TX_QUEUE {
    _opaque: [u8; 128],
    pub tx_queue_name: *mut CHAR,
    pub tx_queue_enqueued: ULONG,
    pub tx_queue_available_storage: ULONG,
}

impl Default for TX_QUEUE {
    fn default() -> Self {
        Self {
            _opaque: [0; 128],
            tx_queue_name: core::ptr::null_mut(),
            tx_queue_enqueued: 0,
            tx_queue_available_storage: 0,
        }
    }
}

// Stub FFI functions that always succeed (for type checking only)

pub unsafe fn _tx_thread_create(
    _thread_ptr: *mut TX_THREAD,
    _name_ptr: *mut CHAR,
    _entry_function: Option<unsafe extern "C" fn(ULONG)>,
    _entry_input: ULONG,
    _stack_start: *mut core::ffi::c_void,
    _stack_size: ULONG,
    _priority: UINT,
    _preempt_threshold: UINT,
    _time_slice: ULONG,
    _auto_start: UINT,
) -> UINT {
    TX_SUCCESS
}

pub unsafe fn _tx_thread_suspend(_thread_ptr: *mut TX_THREAD) -> UINT {
    TX_SUCCESS
}

pub unsafe fn _tx_thread_resume(_thread_ptr: *mut TX_THREAD) -> UINT {
    TX_SUCCESS
}

pub unsafe fn _tx_thread_terminate(_thread_ptr: *mut TX_THREAD) -> UINT {
    TX_SUCCESS
}

pub unsafe fn _tx_thread_reset(_thread_ptr: *mut TX_THREAD) -> UINT {
    TX_SUCCESS
}

pub unsafe fn _tx_thread_delete(_thread_ptr: *mut TX_THREAD) -> UINT {
    TX_SUCCESS
}

pub unsafe fn _tx_thread_sleep(_timer_ticks: ULONG) {
    // No-op
}

pub unsafe fn _tx_thread_relinquish() {
    // No-op
}

pub unsafe fn _tx_thread_identify() -> *mut TX_THREAD {
    core::ptr::null_mut()
}

pub unsafe fn _tx_mutex_create(
    _mutex_ptr: *mut TX_MUTEX,
    _name_ptr: *mut CHAR,
    _inherit: UINT,
) -> UINT {
    TX_SUCCESS
}

pub unsafe fn _tx_mutex_get(_mutex_ptr: *mut TX_MUTEX, _wait_option: ULONG) -> UINT {
    TX_SUCCESS
}

pub unsafe fn _tx_mutex_put(_mutex_ptr: *mut TX_MUTEX) -> UINT {
    TX_SUCCESS
}

pub unsafe fn _tx_mutex_delete(_mutex_ptr: *mut TX_MUTEX) -> UINT {
    TX_SUCCESS
}

pub unsafe fn _tx_queue_create(
    _queue_ptr: *mut TX_QUEUE,
    _name_ptr: *mut CHAR,
    _message_size: UINT,
    _queue_start: *mut core::ffi::c_void,
    _queue_size: ULONG,
) -> UINT {
    TX_SUCCESS
}

pub unsafe fn _tx_queue_send(
    _queue_ptr: *mut TX_QUEUE,
    _source_ptr: *mut core::ffi::c_void,
    _wait_option: ULONG,
) -> UINT {
    TX_SUCCESS
}

pub unsafe fn _tx_queue_front_send(
    _queue_ptr: *mut TX_QUEUE,
    _source_ptr: *mut core::ffi::c_void,
    _wait_option: ULONG,
) -> UINT {
    TX_SUCCESS
}

pub unsafe fn _tx_queue_receive(
    _queue_ptr: *mut TX_QUEUE,
    _destination_ptr: *mut core::ffi::c_void,
    _wait_option: ULONG,
) -> UINT {
    TX_SUCCESS
}

pub unsafe fn _tx_queue_flush(_queue_ptr: *mut TX_QUEUE) -> UINT {
    TX_SUCCESS
}

pub unsafe fn _tx_queue_delete(_queue_ptr: *mut TX_QUEUE) -> UINT {
    TX_SUCCESS
}
