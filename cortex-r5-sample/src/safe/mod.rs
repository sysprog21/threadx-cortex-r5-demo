//! Type-safe ThreadX abstractions using the Context + Resource pattern.
//!
//! This module provides safe wrappers around ThreadX primitives that separate storage
//! (Context) from handles (Resource), enabling compile-time prevention of common errors:
//!
//! - **Use-after-free**: Lifetimes tie handles to their backing storage
//! - **Resource movement**: Pinning prevents control blocks from moving after registration
//! - **Double-creation**: Runtime tracking prevents creating resources twice
//!
//! # Design Pattern
//!
//! Each ThreadX resource follows the same pattern:
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │ Context (owns storage)                                      │
//! │  - Contains ThreadX control block (TX_THREAD, TX_MUTEX, etc)│
//! │  - Must be pinned before use                                │
//! │  - Tracks creation state                                    │
//! │  - Provides try_created() to access resource                │
//! └─────────────────────────────────────────────────────────────┘
//!                              │
//!                              │ create() returns
//!                              ▼
//! ┌─────────────────────────────────────────────────────────────┐
//! │ Resource (safe handle)                                      │
//! │  - Borrows pointer to control block                         │
//! │  - Provides safe API (suspend, lock, send, etc.)            │
//! │  - Lifetime tied to Context                                 │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Usage
//!
//! ```no_run
//! use cortex_r5_sample::safe::thread::{Thread, ThreadContext, ThreadOptions};
//! use static_cell::StaticCell;
//! use core::pin::Pin;
//!
//! // Allocate context in static storage (required for ThreadX)
//! static THREAD_CTX: StaticCell<ThreadContext> = StaticCell::new(ThreadContext::new());
//! static STACK: StaticCell<[u8; 4096]> = StaticCell::new([0u8; 4096]);
//!
//! // Define entry point
//! unsafe extern "C" fn my_thread_entry(_: u32) {
//!     loop {
//!         threadx_sys::_tx_thread_sleep(100);
//!     }
//! }
//!
//! // In tx_application_define():
//! fn create_thread() {
//!     let ctx = THREAD_CTX.uninit();
//!     let stack = STACK.uninit();
//!
//!     // Pin the context and create the thread
//!     let pinned = unsafe { Pin::new_unchecked(ctx.assume_init_mut()) };
//!     let thread = Thread::create(
//!         pinned,
//!         c"my-thread",
//!         unsafe { stack.assume_init_mut() },
//!         ThreadOptions {
//!             entry_fn: my_thread_entry,
//!             priority: 10,
//!             ..Default::default()
//!         },
//!     ).expect("Failed to create thread");
//! }
//! ```
//!
//! # Safety Guarantees
//!
//! ## Pinning
//!
//! ThreadX maintains internal linked lists of pointers to control blocks. If a control
//! block moves after registration, these pointers become dangling. The `PhantomPinned`
//! marker on all Context types prevents moving via Rust's `!Unpin` trait.
//!
//! ## Lifetime Invariance
//!
//! The `InvariantLifetime<'ctx>` marker prevents lifetime covariance issues that could
//! allow a longer-lived reference where a shorter-lived one is expected.
//!
//! ## Creation Tracking
//!
//! Each context tracks whether its resource has been created, preventing double-creation
//! which would corrupt ThreadX's internal state.

#![allow(dead_code)]

use core::cell::Cell;
use core::marker::PhantomData;

pub mod mutex;
pub mod queue;
pub mod thread;

// Re-export main types for convenience
pub use mutex::{Mutex, MutexContext, MutexGuard};
pub use queue::{queue_storage_size, Queue, QueueContext};
pub use thread::{Thread, ThreadContext, ThreadOptions};

/// Marker for invariant lifetime tracking.
///
/// This type forces the lifetime `'ctx` to be invariant, meaning it cannot be
/// shortened or lengthened. This is crucial for ThreadX resources where the
/// borrowed data (names, storage) must remain valid for exactly the resource's
/// lifetime.
///
/// Invariance prevents subtle lifetime bugs like:
/// - Covariant shrinking that could allow dangling references
/// - Contravariant widening that could violate borrow checker assumptions
pub struct InvariantLifetime<'ctx> {
    /// Cell<&'ctx ()> is invariant over 'ctx
    _invariant: PhantomData<Cell<&'ctx ()>>,
}

impl<'ctx> InvariantLifetime<'ctx> {
    /// Create a new invariant lifetime marker.
    pub const fn new() -> Self {
        Self {
            _invariant: PhantomData,
        }
    }
}

impl<'ctx> Default for InvariantLifetime<'ctx> {
    fn default() -> Self {
        Self::new()
    }
}

/// Common error type for ThreadX operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TxError {
    /// ThreadX returned an error code
    ThreadXError(u32),
    /// Resource was already created
    AlreadyCreated,
    /// Invalid parameter
    InvalidParameter,
    /// Resource not created yet
    NotCreated,
    /// Operation timed out
    Timeout,
    /// Caller is in wrong context (e.g., ISR calling blocking function)
    InvalidCaller,
}

impl TxError {
    /// Convert ThreadX status code to Result.
    pub fn from_status(status: u32) -> Result<(), Self> {
        if status == threadx_sys::TX_SUCCESS {
            Ok(())
        } else {
            Err(TxError::ThreadXError(status))
        }
    }
}

// Note: Common context methods (is_created, try_created, assume_created) are
// implemented directly in each context type rather than via macro, because
// Rust macros have difficulty with lifetime-generic types like ThreadContext<'ctx>.
