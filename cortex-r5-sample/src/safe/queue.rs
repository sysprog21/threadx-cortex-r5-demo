//! Type-safe ThreadX queue abstraction using Context + Resource pattern.
//!
//! This module provides safe wrappers around ThreadX queue primitives for
//! inter-thread communication.
//!
//! # Pattern Overview
//!
//! ```text
//! QueueContext<T> (owns TX_QUEUE control block)
//!       │
//!       │ Queue::create(Pin<&QueueContext>, storage, ...)
//!       ▼
//! &Queue<T> (safe handle for send/receive)
//! ```
//!
//! # Safety Notes
//!
//! ThreadX queues operate on 32-bit words (`ULONG`). This wrapper handles:
//! - **Alignment**: Storage must be `ULONG` array to ensure 4-byte alignment.
//! - **Padding**: Messages are copied to/from intermediate buffers to prevent over-reads/writes.
//! - **Sizing**: Use [`queue_storage_size`] to calculate required storage size.
//!
//! # Example
//!
//! ```no_run
//! use cortex_r5_sample::safe::queue::{Queue, QueueContext, QueueOptions, queue_storage_size};
//! use static_cell::StaticCell;
//! use threadx_sys::ULONG;
//! use core::pin::Pin;
//!
//! // Queue context and storage for 16 u32 messages
//! static QUEUE_CTX: StaticCell<QueueContext<u32>> = StaticCell::new(QueueContext::new());
//! static STORAGE: StaticCell<[ULONG; queue_storage_size::<u32>(16)]> =
//!     StaticCell::new([0; queue_storage_size::<u32>(16)]);
//!
//! // In tx_application_define:
//! let ctx = unsafe { QUEUE_CTX.uninit().assume_init_mut() };
//! let storage = unsafe { STORAGE.uninit().assume_init_mut() };
//! let pinned = unsafe { Pin::new_unchecked(ctx) };
//!
//! let queue = Queue::create(pinned, c"msg-queue", storage, QueueOptions::default())
//!     .expect("queue create failed");
//!
//! // In producer thread:
//! queue.send(&42u32).expect("send failed");
//!
//! // In consumer thread:
//! let mut msg = 0u32;
//! queue.receive(&mut msg).expect("receive failed");
//! ```

use core::cell::UnsafeCell;
use core::ffi::CStr;
use core::marker::{PhantomData, PhantomPinned};
use core::mem;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, Ordering};

use super::TxError;
use threadx_sys::{CHAR, TX_QUEUE, UINT, ULONG};

/// Errors that can occur during queue creation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CreateError {
    /// ThreadX returned an error code
    ThreadXError(UINT),
    /// Queue already created with this context
    AlreadyCreated,
    /// Invalid caller (e.g., from ISR)
    InvalidCaller,
    /// Invalid message size
    InvalidSize,
}

impl From<CreateError> for TxError {
    fn from(e: CreateError) -> Self {
        match e {
            CreateError::ThreadXError(code) => TxError::ThreadXError(code),
            CreateError::AlreadyCreated => TxError::AlreadyCreated,
            CreateError::InvalidCaller => TxError::InvalidCaller,
            CreateError::InvalidSize => TxError::InvalidParameter,
        }
    }
}

/// Errors that can occur during queue operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QueueError {
    /// Queue is full (send failed)
    Full,
    /// Queue is empty (receive failed)
    Empty,
    /// Operation timed out
    Timeout,
    /// Invalid caller
    InvalidCaller,
    /// Queue was deleted while waiting
    Deleted,
    /// Wait was aborted
    WaitAborted,
    /// ThreadX error
    ThreadXError(UINT),
}

/// Options for configuring queue behavior.
#[derive(Clone, Copy, Default)]
pub struct QueueOptions {
    // Currently no options, but placeholder for future extensions
    _reserved: (),
}

/// Helper constant to calculate message size in words.
const fn msg_words<T>() -> usize {
    (mem::size_of::<T>() + 3) / 4
}

/// Queue context storage (owns the ThreadX control block).
///
/// The type parameter `T` is the message type. Storage is provided externally
/// during creation, similar to how thread stacks are provided externally.
///
/// # Type Requirements
///
/// `T` must be `Copy` because:
/// - Messages are copied byte-by-byte through ThreadX
/// - Non-Copy types could have drop glue or internal references that would be corrupted
/// - This prevents use-after-free and double-free bugs
///
/// # Storage Requirements
///
/// Storage must be:
/// - 4-byte aligned (use `[ULONG; N * words_per_message]`)
/// - Large enough for the desired capacity
/// - `'static` lifetime (ThreadX holds pointers to it)
///
/// Use [`queue_storage_size`] to calculate the required storage size.
#[repr(C)]
pub struct QueueContext<T: Copy> {
    /// The wrapped Queue resource
    queue: Queue<T>,
    /// Tracks whether the queue has been created
    created: AtomicBool,
}

// QueueContext is Sync because we use AtomicBool and Queue is Sync.
unsafe impl<T: Copy + Send> Sync for QueueContext<T> {}

impl<T: Copy> QueueContext<T> {
    /// Creates a new queue context.
    ///
    /// Storage must be provided separately during `Queue::create()`.
    pub const fn new() -> Self {
        Self {
            queue: Queue::new(),
            created: AtomicBool::new(false),
        }
    }

    /// Returns whether the queue has been created.
    #[inline]
    pub fn is_created(&self) -> bool {
        self.created.load(Ordering::Acquire)
    }

    /// Returns the queue if created.
    #[inline]
    pub fn try_created(self: Pin<&Self>) -> Option<&Queue<T>> {
        if self.is_created() {
            Some(unsafe { self.assume_created() })
        } else {
            None
        }
    }

    /// Returns the queue without checking creation status.
    ///
    /// # Safety
    ///
    /// The queue must have been successfully created.
    #[inline]
    pub unsafe fn assume_created(self: Pin<&Self>) -> &Queue<T> {
        &self.get_ref().queue
    }

    /// Resets the created flag after queue deletion.
    ///
    /// # Safety
    ///
    /// Only call after the queue has been deleted.
    pub unsafe fn reset_created(&self) {
        self.created.store(false, Ordering::Release);
    }
}

/// Calculate the storage size in ULONGs needed for a queue.
///
/// # Arguments
///
/// - `capacity`: Number of messages the queue should hold
///
/// # Example
///
/// ```no_run
/// use cortex_r5_sample::safe::queue::queue_storage_size;
/// use threadx_sys::ULONG;
///
/// // Storage for 16 u32 messages
/// static STORAGE: [ULONG; queue_storage_size::<u32>(16)] = [0; queue_storage_size::<u32>(16)];
/// ```
pub const fn queue_storage_size<T>(capacity: usize) -> usize {
    msg_words::<T>() * capacity
}

/// Queue resource handle.
///
/// Provides thread-safe message passing between threads.
///
/// # Type Requirements
///
/// `T` must be `Copy` to ensure safe byte-by-byte message transfer through ThreadX.
#[repr(transparent)]
pub struct Queue<T: Copy> {
    /// ThreadX control block
    control_block: UnsafeCell<TX_QUEUE>,
    /// Prevents moving
    _pin: PhantomPinned,
    /// Marker for message type
    _marker: PhantomData<T>,
}

// Queue is Sync because ThreadX APIs use internal critical sections
unsafe impl<T: Copy + Send> Sync for Queue<T> {}

impl<T: Copy> Queue<T> {
    /// Creates a zeroed Queue (internal use only).
    const fn new() -> Self {
        Self {
            control_block: UnsafeCell::new(unsafe { mem::zeroed() }),
            _pin: PhantomPinned,
            _marker: PhantomData,
        }
    }

    /// Get raw pointer to control block.
    #[inline]
    fn as_ptr(&self) -> *mut TX_QUEUE {
        self.control_block.get()
    }

    /// Calculates the message size in 32-bit words.
    const fn message_size_words() -> usize {
        msg_words::<T>()
    }

    /// Creates and initializes a ThreadX queue.
    ///
    /// # Arguments
    ///
    /// - `context`: Pinned context storage
    /// - `name`: Null-terminated queue name
    /// - `storage`: Message storage buffer (must be 4-byte aligned ULONG array)
    /// - `_options`: Queue configuration (currently unused)
    ///
    /// # Storage Requirements
    ///
    /// Storage must be large enough for the desired capacity:
    /// - Size in ULONGs = capacity * `queue_storage_size::<T>(1)`
    /// - Must be `'static` lifetime (ThreadX holds pointers)
    ///
    /// # Returns
    ///
    /// A reference to the `Queue` resource.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use cortex_r5_sample::safe::queue::{Queue, QueueContext, QueueOptions, queue_storage_size};
    /// use static_cell::StaticCell;
    /// use threadx_sys::ULONG;
    /// use core::pin::Pin;
    ///
    /// static QUEUE_CTX: StaticCell<QueueContext<u32>> = StaticCell::new(QueueContext::new());
    /// static STORAGE: StaticCell<[ULONG; queue_storage_size::<u32>(16)]> =
    ///     StaticCell::new([0; queue_storage_size::<u32>(16)]);
    ///
    /// let ctx = unsafe { QUEUE_CTX.uninit().assume_init_mut() };
    /// let storage = unsafe { STORAGE.uninit().assume_init_mut() };
    /// let pinned = unsafe { Pin::new_unchecked(ctx) };
    ///
    /// let queue = Queue::create(pinned, c"msg-queue", storage, QueueOptions::default())
    ///     .expect("queue create failed");
    /// ```
    pub fn create<'t>(
        context: Pin<&'t QueueContext<T>>,
        name: &'static CStr,
        storage: &'static mut [ULONG],
        _options: QueueOptions,
    ) -> Result<&'t Queue<T>, CreateError> {
        // Atomically claim the context to prevent race conditions.
        // compare_exchange ensures only one thread can successfully claim.
        if context
            .created
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Relaxed)
            .is_err()
        {
            return Err(CreateError::AlreadyCreated);
        }

        let msg_size = Self::message_size_words();

        // ThreadX limits message size to 16 words (64 bytes)
        if msg_size > 16 || msg_size == 0 {
            // Roll back - allow re-creation attempts
            context.created.store(false, Ordering::Release);
            return Err(CreateError::InvalidSize);
        }

        // Validate storage size: must hold at least one message and be
        // an exact multiple of message size to avoid truncation issues
        if storage.is_empty() || storage.len() < msg_size || storage.len() % msg_size != 0 {
            context.created.store(false, Ordering::Release);
            return Err(CreateError::InvalidSize);
        }

        let storage_ptr = storage.as_mut_ptr() as *mut core::ffi::c_void;
        let storage_size_bytes = storage.len() * mem::size_of::<ULONG>();

        let result = unsafe {
            threadx_sys::_tx_queue_create(
                context.queue.as_ptr(),
                name.as_ptr() as *mut CHAR,
                msg_size as UINT,
                storage_ptr,
                storage_size_bytes as ULONG,
            )
        };

        match result {
            threadx_sys::TX_SUCCESS => Ok(&context.get_ref().queue),
            threadx_sys::TX_CALLER_ERROR => {
                // Roll back - allow re-creation attempts
                context.created.store(false, Ordering::Release);
                Err(CreateError::InvalidCaller)
            }
            _ => {
                // Roll back - allow re-creation attempts
                context.created.store(false, Ordering::Release);
                Err(CreateError::ThreadXError(result))
            }
        }
    }

    /// Sends a message to the queue, blocking until space is available.
    pub fn send(&self, message: &T) -> Result<(), QueueError> {
        self.send_with_timeout(message, threadx_sys::TX_WAIT_FOREVER)
    }

    /// Attempts to send a message without blocking.
    pub fn try_send(&self, message: &T) -> Result<(), QueueError> {
        self.send_with_timeout(message, threadx_sys::TX_NO_WAIT)
    }

    /// Sends a message with a timeout.
    pub fn send_with_timeout(&self, message: &T, timeout: ULONG) -> Result<(), QueueError> {
        // Copy message to aligned stack buffer to prevent ThreadX over-read
        let mut buffer = [0u32; 16]; // Max message size is 16 words
        let words = Self::message_size_words();

        // SAFETY: Message size is validated in Queue::create() - this is a sanity check.
        // T: Copy bound ensures safe byte-by-byte transfer.
        debug_assert!(
            words <= 16,
            "Message too large - should be caught in create()"
        );

        // Copy T into buffer
        // We use ptr::copy_nonoverlapping to treat T as bytes and copy to ULONG array
        unsafe {
            let src = message as *const T as *const u8;
            let dst = buffer.as_mut_ptr() as *mut u8;
            core::ptr::copy_nonoverlapping(src, dst, mem::size_of::<T>());
        }

        let result = unsafe {
            threadx_sys::_tx_queue_send(
                self.as_ptr(),
                buffer.as_ptr() as *mut core::ffi::c_void,
                timeout,
            )
        };

        Self::map_send_result(result, timeout)
    }

    /// Sends a message to the front of the queue (highest priority).
    pub fn send_front(&self, message: &T) -> Result<(), QueueError> {
        self.send_front_with_timeout(message, threadx_sys::TX_WAIT_FOREVER)
    }

    /// Sends to front without blocking.
    pub fn try_send_front(&self, message: &T) -> Result<(), QueueError> {
        self.send_front_with_timeout(message, threadx_sys::TX_NO_WAIT)
    }

    /// Sends to front with timeout.
    pub fn send_front_with_timeout(&self, message: &T, timeout: ULONG) -> Result<(), QueueError> {
        let mut buffer = [0u32; 16];
        let words = Self::message_size_words();
        debug_assert!(
            words <= 16,
            "Message too large - should be caught in create()"
        );

        unsafe {
            let src = message as *const T as *const u8;
            let dst = buffer.as_mut_ptr() as *mut u8;
            core::ptr::copy_nonoverlapping(src, dst, mem::size_of::<T>());
        }

        let result = unsafe {
            threadx_sys::_tx_queue_front_send(
                self.as_ptr(),
                buffer.as_ptr() as *mut core::ffi::c_void,
                timeout,
            )
        };

        Self::map_send_result(result, timeout)
    }

    /// Helper to map ThreadX send results
    fn map_send_result(result: UINT, timeout: ULONG) -> Result<(), QueueError> {
        match result {
            threadx_sys::TX_SUCCESS => Ok(()),
            threadx_sys::TX_QUEUE_FULL => Err(QueueError::Full),
            threadx_sys::TX_WAIT_ABORTED => Err(QueueError::WaitAborted),
            threadx_sys::TX_QUEUE_ERROR => Err(QueueError::Deleted),
            threadx_sys::TX_CALLER_ERROR => Err(QueueError::InvalidCaller),
            _ => {
                if timeout != threadx_sys::TX_WAIT_FOREVER && timeout != threadx_sys::TX_NO_WAIT {
                    Err(QueueError::Timeout)
                } else {
                    Err(QueueError::ThreadXError(result))
                }
            }
        }
    }

    /// Receives a message from the queue, blocking until one is available.
    pub fn receive(&self, dest: &mut T) -> Result<(), QueueError> {
        self.receive_with_timeout(dest, threadx_sys::TX_WAIT_FOREVER)
    }

    /// Attempts to receive a message without blocking.
    pub fn try_receive(&self, dest: &mut T) -> Result<(), QueueError> {
        self.receive_with_timeout(dest, threadx_sys::TX_NO_WAIT)
    }

    /// Receives a message with a timeout.
    pub fn receive_with_timeout(&self, dest: &mut T, timeout: ULONG) -> Result<(), QueueError> {
        let mut buffer = [0u32; 16];
        let words = Self::message_size_words();
        debug_assert!(
            words <= 16,
            "Message too large - should be caught in create()"
        );

        let result = unsafe {
            threadx_sys::_tx_queue_receive(
                self.as_ptr(),
                buffer.as_mut_ptr() as *mut core::ffi::c_void,
                timeout,
            )
        };

        if result == threadx_sys::TX_SUCCESS {
            // Copy from buffer to dest
            unsafe {
                let src = buffer.as_ptr() as *const u8;
                let dst = dest as *mut T as *mut u8;
                core::ptr::copy_nonoverlapping(src, dst, mem::size_of::<T>());
            }
            Ok(())
        } else {
            Self::map_receive_result(result, timeout)
        }
    }

    /// Helper to map ThreadX receive results
    fn map_receive_result(result: UINT, timeout: ULONG) -> Result<(), QueueError> {
        match result {
            threadx_sys::TX_SUCCESS => Ok(()), // Should be handled by caller
            threadx_sys::TX_QUEUE_EMPTY => Err(QueueError::Empty),
            threadx_sys::TX_WAIT_ABORTED => Err(QueueError::WaitAborted),
            threadx_sys::TX_QUEUE_ERROR => Err(QueueError::Deleted),
            threadx_sys::TX_CALLER_ERROR => Err(QueueError::InvalidCaller),
            _ => {
                if timeout != threadx_sys::TX_WAIT_FOREVER && timeout != threadx_sys::TX_NO_WAIT {
                    Err(QueueError::Timeout)
                } else {
                    Err(QueueError::ThreadXError(result))
                }
            }
        }
    }

    /// Flushes all messages from the queue.
    pub fn flush(&self) -> Result<(), UINT> {
        let result = unsafe { threadx_sys::_tx_queue_flush(self.as_ptr()) };
        if result == threadx_sys::TX_SUCCESS {
            Ok(())
        } else {
            Err(result)
        }
    }

    /// Deletes the queue, removing it from ThreadX.
    ///
    /// Any threads waiting on the queue will receive an error.
    pub fn delete(&self) -> Result<(), UINT> {
        let result = unsafe { threadx_sys::_tx_queue_delete(self.as_ptr()) };
        if result == threadx_sys::TX_SUCCESS {
            Ok(())
        } else {
            Err(result)
        }
    }

    /// Gets the queue's name.
    ///
    /// # Safety
    ///
    /// This reads from the ThreadX control block.
    pub unsafe fn name(&self) -> Option<&'static str> {
        let name_ptr = (*self.as_ptr()).tx_queue_name;
        if name_ptr.is_null() {
            return None;
        }
        let c_str = CStr::from_ptr(name_ptr);
        c_str.to_str().ok()
    }

    /// Returns the number of messages currently in the queue.
    ///
    /// # Safety
    ///
    /// This reads from the ThreadX control block.
    pub unsafe fn enqueued(&self) -> ULONG {
        (*self.as_ptr()).tx_queue_enqueued
    }

    /// Returns the available capacity (messages that can still be sent).
    ///
    /// # Safety
    ///
    /// This reads from the ThreadX control block.
    pub unsafe fn available_storage(&self) -> ULONG {
        (*self.as_ptr()).tx_queue_available_storage
    }
}
