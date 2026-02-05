//! Centralized ThreadX status mapping and error handling.
//!
//! This module provides type-safe error handling for ThreadX API calls,
//! mapping raw `UINT` status codes to a Rust enum with descriptive variants.
//!
//! # Usage
//!
//! ```ignore
//! use cortex_r5_sample::threadx_error::{TxResult, TxStatus, tx_check};
//!
//! // Using tx_check! macro for simple calls
//! fn create_thread(thread: *mut threadx_sys::TX_THREAD) -> TxResult<()> {
//!     unsafe {
//!         tx_check!(threadx_sys::_tx_thread_suspend(thread))
//!     }
//! }
//!
//! // Manual status checking
//! fn manual_check(status: u32) -> TxResult<()> {
//!     TxStatus::from_code(status).into_result()
//! }
//! ```
//!
//! # Error Codes Reference
//!
//! | Code | Variant | Description |
//! |------|---------|-------------|
//! | 0x00 | Success | Operation completed successfully |
//! | 0x01 | Deleted | Resource was deleted during operation |
//! | 0x02 | PoolError | Invalid memory pool pointer |
//! | 0x03 | PtrError | Invalid pointer parameter |
//! | 0x04 | WaitError | Invalid wait option |
//! | 0x05 | SizeError | Invalid size parameter |
//! | 0x06 | GroupError | Invalid event flags group |
//! | 0x07 | NoEvents | Requested events not present |
//! | 0x08 | OptionError | Invalid option parameter |
//! | 0x09 | QueueError | Invalid queue pointer |
//! | 0x0A | QueueEmpty | Queue has no messages |
//! | 0x0B | QueueFull | Queue has no space |
//! | 0x0C | SemaphoreError | Invalid semaphore pointer |
//! | 0x0D | NoInstance | Resource not available |
//! | 0x0E | ThreadError | Invalid thread pointer |
//! | 0x0F | PriorityError | Invalid priority value |
//! | 0x10 | NoMemory | Insufficient memory |
//! | 0x11 | DeleteError | Thread not in terminatable state |
//! | 0x12 | ResumeError | Thread not suspended |
//! | 0x13 | CallerError | Invalid caller context (e.g., ISR) |
//! | 0x14 | SuspendError | Thread not in suspendable state |
//! | 0x15 | TimerError | Invalid timer pointer |
//! | 0x16 | TickError | Invalid tick value |
//! | 0x17 | ActivateError | Timer already active |
//! | 0x18 | ThreshError | Invalid preemption threshold |
//! | 0x19 | SuspendLifted | Suspension was lifted |
//! | 0x1A | WaitAborted | Wait was aborted |
//! | 0x1B | WaitAbortError | Cannot abort wait |
//! | 0x1C | MutexError | Invalid mutex pointer |
//! | 0x1D | NotAvailable | Resource not available (try_lock) |
//! | 0x1E | NotOwned | Mutex not owned by caller |
//! | 0x1F | InheritError | Priority inheritance error |
//! | 0x20 | NotDone | Operation not complete |
//! | 0x21 | CeilingExceeded | Priority ceiling exceeded |
//! | 0x22 | InvalidCeiling | Invalid ceiling value |
//! | 0xFF | FeatureNotEnabled | Feature not compiled in |

/// ThreadX status codes mapped to a Rust enum.
///
/// This enum represents all possible return values from ThreadX API functions.
/// The discriminant values match the ThreadX `TX_*` constants exactly.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u32)]
pub enum TxStatus {
    /// Operation completed successfully (TX_SUCCESS = 0x00)
    Success = 0x00,
    /// Resource was deleted during wait (TX_DELETED = 0x01)
    Deleted = 0x01,
    /// Invalid memory pool pointer (TX_POOL_ERROR = 0x02)
    PoolError = 0x02,
    /// Invalid pointer parameter (TX_PTR_ERROR = 0x03)
    PtrError = 0x03,
    /// Invalid wait option specified (TX_WAIT_ERROR = 0x04)
    WaitError = 0x04,
    /// Invalid size parameter (TX_SIZE_ERROR = 0x05)
    SizeError = 0x05,
    /// Invalid event flags group (TX_GROUP_ERROR = 0x06)
    GroupError = 0x06,
    /// Requested events not present (TX_NO_EVENTS = 0x07)
    NoEvents = 0x07,
    /// Invalid option parameter (TX_OPTION_ERROR = 0x08)
    OptionError = 0x08,
    /// Invalid queue pointer (TX_QUEUE_ERROR = 0x09)
    QueueError = 0x09,
    /// Queue is empty, no messages (TX_QUEUE_EMPTY = 0x0A)
    QueueEmpty = 0x0A,
    /// Queue is full, cannot send (TX_QUEUE_FULL = 0x0B)
    QueueFull = 0x0B,
    /// Invalid semaphore pointer (TX_SEMAPHORE_ERROR = 0x0C)
    SemaphoreError = 0x0C,
    /// No instance available (TX_NO_INSTANCE = 0x0D)
    NoInstance = 0x0D,
    /// Invalid thread pointer (TX_THREAD_ERROR = 0x0E)
    ThreadError = 0x0E,
    /// Invalid priority value (TX_PRIORITY_ERROR = 0x0F)
    PriorityError = 0x0F,
    /// Insufficient memory (TX_NO_MEMORY = 0x10)
    ///
    /// Note: ThreadX also defines TX_START_ERROR = 0x10 with the same value.
    /// Context determines whether this means "no memory" or "start error".
    NoMemory = 0x10,
    /// Thread not terminatable (TX_DELETE_ERROR = 0x11)
    DeleteError = 0x11,
    /// Thread not suspended (TX_RESUME_ERROR = 0x12)
    ResumeError = 0x12,
    /// Invalid caller context, e.g., ISR (TX_CALLER_ERROR = 0x13)
    CallerError = 0x13,
    /// Thread cannot be suspended (TX_SUSPEND_ERROR = 0x14)
    SuspendError = 0x14,
    /// Invalid timer pointer (TX_TIMER_ERROR = 0x15)
    TimerError = 0x15,
    /// Invalid tick value (TX_TICK_ERROR = 0x16)
    TickError = 0x16,
    /// Timer already active (TX_ACTIVATE_ERROR = 0x17)
    ActivateError = 0x17,
    /// Invalid preemption threshold (TX_THRESH_ERROR = 0x18)
    ThreshError = 0x18,
    /// Suspension was lifted (TX_SUSPEND_LIFTED = 0x19)
    SuspendLifted = 0x19,
    /// Wait was aborted (TX_WAIT_ABORTED = 0x1A)
    WaitAborted = 0x1A,
    /// Cannot abort wait (TX_WAIT_ABORT_ERROR = 0x1B)
    WaitAbortError = 0x1B,
    /// Invalid mutex pointer (TX_MUTEX_ERROR = 0x1C)
    MutexError = 0x1C,
    /// Resource not available (TX_NOT_AVAILABLE = 0x1D)
    NotAvailable = 0x1D,
    /// Mutex not owned by caller (TX_NOT_OWNED = 0x1E)
    NotOwned = 0x1E,
    /// Priority inheritance error (TX_INHERIT_ERROR = 0x1F)
    InheritError = 0x1F,
    /// Operation not complete (TX_NOT_DONE = 0x20)
    NotDone = 0x20,
    /// Priority ceiling exceeded (TX_CEILING_EXCEEDED = 0x21)
    CeilingExceeded = 0x21,
    /// Invalid priority ceiling (TX_INVALID_CEILING = 0x22)
    InvalidCeiling = 0x22,
    /// Feature not enabled in build (TX_FEATURE_NOT_ENABLED = 0xFF)
    FeatureNotEnabled = 0xFF,
    /// Unknown error code (for forward compatibility)
    ///
    /// Note: Uses 0xFFFF_FFFF as sentinel. If ThreadX ever returns this exact
    /// value (extremely unlikely), it will be indistinguishable from an unknown code.
    Unknown = 0xFFFF_FFFF,
}

impl TxStatus {
    /// Convert a raw ThreadX status code to `TxStatus`.
    ///
    /// Unknown codes map to `TxStatus::Unknown` for forward compatibility.
    #[inline]
    pub const fn from_code(code: u32) -> Self {
        match code {
            0x00 => TxStatus::Success,
            0x01 => TxStatus::Deleted,
            0x02 => TxStatus::PoolError,
            0x03 => TxStatus::PtrError,
            0x04 => TxStatus::WaitError,
            0x05 => TxStatus::SizeError,
            0x06 => TxStatus::GroupError,
            0x07 => TxStatus::NoEvents,
            0x08 => TxStatus::OptionError,
            0x09 => TxStatus::QueueError,
            0x0A => TxStatus::QueueEmpty,
            0x0B => TxStatus::QueueFull,
            0x0C => TxStatus::SemaphoreError,
            0x0D => TxStatus::NoInstance,
            0x0E => TxStatus::ThreadError,
            0x0F => TxStatus::PriorityError,
            0x10 => TxStatus::NoMemory,
            0x11 => TxStatus::DeleteError,
            0x12 => TxStatus::ResumeError,
            0x13 => TxStatus::CallerError,
            0x14 => TxStatus::SuspendError,
            0x15 => TxStatus::TimerError,
            0x16 => TxStatus::TickError,
            0x17 => TxStatus::ActivateError,
            0x18 => TxStatus::ThreshError,
            0x19 => TxStatus::SuspendLifted,
            0x1A => TxStatus::WaitAborted,
            0x1B => TxStatus::WaitAbortError,
            0x1C => TxStatus::MutexError,
            0x1D => TxStatus::NotAvailable,
            0x1E => TxStatus::NotOwned,
            0x1F => TxStatus::InheritError,
            0x20 => TxStatus::NotDone,
            0x21 => TxStatus::CeilingExceeded,
            0x22 => TxStatus::InvalidCeiling,
            0xFF => TxStatus::FeatureNotEnabled,
            _ => TxStatus::Unknown,
        }
    }

    /// Returns `true` if this status indicates success.
    #[inline]
    pub const fn is_success(self) -> bool {
        matches!(self, TxStatus::Success)
    }

    /// Returns `true` if this status indicates an error.
    #[inline]
    pub const fn is_error(self) -> bool {
        !self.is_success()
    }

    /// Convert to `Result<(), TxStatus>`.
    ///
    /// Returns `Ok(())` for `Success`, `Err(self)` for any error.
    #[inline]
    pub const fn into_result(self) -> TxResult<()> {
        if self.is_success() {
            Ok(())
        } else {
            Err(self)
        }
    }

    /// Get the raw numeric code.
    #[inline]
    pub const fn code(self) -> u32 {
        self as u32
    }

    /// Get a human-readable description of the status.
    pub const fn description(self) -> &'static str {
        match self {
            TxStatus::Success => "operation completed successfully",
            TxStatus::Deleted => "resource was deleted during wait",
            TxStatus::PoolError => "invalid memory pool pointer",
            TxStatus::PtrError => "invalid pointer parameter",
            TxStatus::WaitError => "invalid wait option specified",
            TxStatus::SizeError => "invalid size parameter",
            TxStatus::GroupError => "invalid event flags group",
            TxStatus::NoEvents => "requested events not present",
            TxStatus::OptionError => "invalid option parameter",
            TxStatus::QueueError => "invalid queue pointer",
            TxStatus::QueueEmpty => "queue is empty",
            TxStatus::QueueFull => "queue is full",
            TxStatus::SemaphoreError => "invalid semaphore pointer",
            TxStatus::NoInstance => "no instance available",
            TxStatus::ThreadError => "invalid thread pointer",
            TxStatus::PriorityError => "invalid priority value",
            TxStatus::NoMemory => "insufficient memory",
            TxStatus::DeleteError => "thread not in terminatable state",
            TxStatus::ResumeError => "thread not suspended",
            TxStatus::CallerError => "invalid caller context (ISR or initialization)",
            TxStatus::SuspendError => "thread cannot be suspended",
            TxStatus::TimerError => "invalid timer pointer",
            TxStatus::TickError => "invalid tick value",
            TxStatus::ActivateError => "timer already active",
            TxStatus::ThreshError => "invalid preemption threshold",
            TxStatus::SuspendLifted => "suspension was lifted",
            TxStatus::WaitAborted => "wait was aborted",
            TxStatus::WaitAbortError => "cannot abort wait",
            TxStatus::MutexError => "invalid mutex pointer",
            TxStatus::NotAvailable => "resource not available",
            TxStatus::NotOwned => "mutex not owned by caller",
            TxStatus::InheritError => "priority inheritance error",
            TxStatus::NotDone => "operation not complete",
            TxStatus::CeilingExceeded => "priority ceiling exceeded",
            TxStatus::InvalidCeiling => "invalid priority ceiling value",
            TxStatus::FeatureNotEnabled => "feature not enabled in build",
            TxStatus::Unknown => "unknown error code",
        }
    }
}

impl From<u32> for TxStatus {
    #[inline]
    fn from(code: u32) -> Self {
        TxStatus::from_code(code)
    }
}

impl From<TxStatus> for u32 {
    #[inline]
    fn from(status: TxStatus) -> Self {
        status.code()
    }
}

/// Result type for ThreadX operations.
///
/// Success returns the value `T`, failure returns `TxStatus`.
///
/// Note: `Result` itself is already `#[must_use]`, so ignoring errors will warn.
pub type TxResult<T> = Result<T, TxStatus>;

/// Check a ThreadX status code and convert to `TxResult<()>`.
///
/// This macro wraps a ThreadX FFI call and converts the return value
/// to a Rust `Result` type.
///
/// # Example
///
/// ```no_run
/// use cortex_r5_sample::threadx_error::{tx_check, TxResult};
///
/// fn suspend_thread(thread: *mut threadx_sys::TX_THREAD) -> TxResult<()> {
///     unsafe { tx_check!(threadx_sys::_tx_thread_suspend(thread)) }
/// }
/// ```
#[macro_export]
macro_rules! tx_check {
    ($call:expr) => {{
        // Explicit type annotation prevents silent truncation of wider types
        let status: u32 = $call;
        $crate::threadx_error::TxStatus::from_code(status).into_result()
    }};
}

/// Check status and return a value on success.
///
/// # Example
///
/// ```ignore
/// use cortex_r5_sample::threadx_error::{tx_check_with, TxResult};
///
/// fn get_value(flags: *mut threadx_sys::TX_EVENT_FLAGS_GROUP) -> TxResult<u32> {
///     let mut actual = 0u32;
///     unsafe {
///         tx_check_with!(threadx_sys::_tx_event_flags_get(
///             flags, 0x1, 0, &mut actual, 0), actual)
///     }
/// }
/// ```
#[macro_export]
macro_rules! tx_check_with {
    ($call:expr, $value:expr) => {{
        // Explicit type annotation prevents silent truncation of wider types
        let status: u32 = $call;
        match $crate::threadx_error::TxStatus::from_code(status).into_result() {
            Ok(()) => Ok($value),
            Err(e) => Err(e),
        }
    }};
}

// Re-export macros at module level
pub use tx_check;
pub use tx_check_with;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_status_from_code() {
        assert_eq!(TxStatus::from_code(0x00), TxStatus::Success);
        assert_eq!(TxStatus::from_code(0x0B), TxStatus::QueueFull);
        assert_eq!(TxStatus::from_code(0x0D), TxStatus::NoInstance);
        assert_eq!(TxStatus::from_code(0x13), TxStatus::CallerError);
        assert_eq!(TxStatus::from_code(0x1A), TxStatus::WaitAborted);
        assert_eq!(TxStatus::from_code(0xFF), TxStatus::FeatureNotEnabled);
        assert_eq!(TxStatus::from_code(0xABCD), TxStatus::Unknown);
    }

    #[test]
    fn test_is_success() {
        assert!(TxStatus::Success.is_success());
        assert!(!TxStatus::QueueFull.is_success());
        assert!(!TxStatus::Unknown.is_success());
    }

    #[test]
    fn test_into_result() {
        assert!(TxStatus::Success.into_result().is_ok());
        assert!(TxStatus::QueueFull.into_result().is_err());
        assert_eq!(
            TxStatus::NoInstance.into_result(),
            Err(TxStatus::NoInstance)
        );
    }

    #[test]
    fn test_code_roundtrip() {
        for code in 0x00..=0x22 {
            let status = TxStatus::from_code(code);
            if status != TxStatus::Unknown {
                assert_eq!(status.code(), code);
            }
        }
        assert_eq!(TxStatus::FeatureNotEnabled.code(), 0xFF);
    }
}
