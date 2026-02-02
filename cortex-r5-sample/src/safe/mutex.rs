//! Type-safe ThreadX mutex abstraction using Context + Resource pattern.
//!
//! This module provides safe wrappers around ThreadX mutex primitives with RAII
//! lock guards that automatically release the mutex when dropped.
//!
//! # Pattern Overview
//!
//! ```text
//! MutexContext (owns TX_MUTEX control block)
//!       │
//!       │ Mutex::create(Pin<&MutexContext>, ...)
//!       ▼
//! &Mutex (safe handle for operations)
//!       │
//!       │ mutex.lock() / mutex.try_lock()
//!       ▼
//! MutexGuard<'_, T> (RAII guard, auto-releases on drop)
//! ```
//!
//! # Example
//!
//! ```no_run
//! use cortex_r5_sample::safe::mutex::{Mutex, MutexContext, MutexOptions};
//! use static_cell::StaticCell;
//! use core::pin::Pin;
//!
//! static MUTEX_CTX: StaticCell<MutexContext> = StaticCell::new(MutexContext::new());
//!
//! // In tx_application_define:
//! let ctx = unsafe { MUTEX_CTX.uninit().assume_init_mut() };
//! let pinned = unsafe { Pin::new_unchecked(ctx) };
//!
//! let mutex = Mutex::create(pinned, c"my-mutex", MutexOptions::default())
//!     .expect("mutex create failed");
//!
//! // In thread:
//! {
//!     let _guard = mutex.lock().expect("lock failed");
//!     // Critical section - mutex auto-released when guard drops
//! }
//! ```

use core::cell::UnsafeCell;
use core::ffi::CStr;
use core::marker::PhantomPinned;
use core::ops::{Deref, DerefMut};
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, Ordering};

use super::tx_ffi;
use super::TxError;
use tx_ffi::{CHAR, TX_MUTEX, UINT, ULONG};

/// Errors that can occur during mutex creation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CreateError {
    /// ThreadX returned an error code
    ThreadXError(UINT),
    /// Mutex already created with this context
    AlreadyCreated,
    /// Invalid caller (e.g., from ISR)
    InvalidCaller,
}

impl From<CreateError> for TxError {
    fn from(e: CreateError) -> Self {
        match e {
            CreateError::ThreadXError(code) => TxError::ThreadXError(code),
            CreateError::AlreadyCreated => TxError::AlreadyCreated,
            CreateError::InvalidCaller => TxError::InvalidCaller,
        }
    }
}

/// Errors that can occur during mutex lock operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LockError {
    /// Mutex is owned by another thread (try_lock failed)
    WouldBlock,
    /// Lock attempt timed out
    Timeout,
    /// Invalid caller (e.g., from ISR for blocking call)
    InvalidCaller,
    /// Mutex was deleted while waiting
    Deleted,
    /// Wait was aborted
    WaitAborted,
    /// ThreadX error
    ThreadXError(UINT),
}

/// Options for configuring mutex behavior.
#[derive(Clone, Copy)]
pub struct MutexOptions {
    /// Enable priority inheritance to prevent priority inversion.
    ///
    /// When enabled, if a high-priority thread blocks on a mutex held by a
    /// lower-priority thread, the lower-priority thread temporarily inherits
    /// the higher priority until it releases the mutex.
    pub priority_inherit: bool,
}

impl Default for MutexOptions {
    fn default() -> Self {
        Self {
            priority_inherit: true, // Default to enabled for safety
        }
    }
}

/// Mutex context storage (owns the ThreadX control block).
///
/// This structure contains the actual ThreadX mutex control block and must
/// remain pinned after creation.
#[repr(C)]
pub struct MutexContext {
    /// The wrapped Mutex resource
    mutex: Mutex,
    /// Tracks whether the mutex has been created
    created: AtomicBool,
}

// MutexContext is Sync because we use AtomicBool and Mutex is Sync.
unsafe impl Sync for MutexContext {}

impl MutexContext {
    /// Creates a new mutex context in uninitialized state.
    pub const fn new() -> Self {
        Self {
            mutex: Mutex::new(),
            created: AtomicBool::new(false),
        }
    }

    /// Resets the created flag after mutex deletion.
    ///
    /// # Safety
    ///
    /// Only call after the mutex has been deleted.
    pub unsafe fn reset_created(&self) {
        self.created.store(false, Ordering::Release);
    }

    /// Returns `true` if this mutex has been created.
    #[inline]
    pub fn is_created(&self) -> bool {
        self.created.load(Ordering::Acquire)
    }

    /// Returns the mutex handle if it has been created.
    #[inline]
    pub fn try_created(self: Pin<&Self>) -> Option<&Mutex> {
        if self.is_created() {
            Some(unsafe { self.assume_created() })
        } else {
            None
        }
    }

    /// Returns the mutex handle without checking creation status.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `Mutex::create()` was called and succeeded.
    #[inline]
    pub unsafe fn assume_created(self: Pin<&Self>) -> &Mutex {
        &self.get_ref().mutex
    }
}

/// Mutex resource handle (safe API for mutex operations).
///
/// Provides mutual exclusion with optional priority inheritance.
#[repr(transparent)]
pub struct Mutex {
    /// ThreadX control block
    control_block: UnsafeCell<TX_MUTEX>,
    /// Prevents moving
    _pin: PhantomPinned,
}

// Mutex is Sync because ThreadX APIs use internal critical sections
unsafe impl Sync for Mutex {}

impl Mutex {
    /// Creates a zeroed Mutex (internal use only).
    const fn new() -> Self {
        Self {
            control_block: UnsafeCell::new(unsafe { core::mem::zeroed() }),
            _pin: PhantomPinned,
        }
    }

    /// Get raw pointer to control block.
    #[inline]
    fn as_ptr(&self) -> *mut TX_MUTEX {
        self.control_block.get()
    }

    /// Creates and initializes a ThreadX mutex.
    ///
    /// # Arguments
    ///
    /// - `context`: Pinned context storage
    /// - `name`: Null-terminated mutex name
    /// - `options`: Mutex configuration
    ///
    /// # Returns
    ///
    /// A reference to the `Mutex` resource.
    pub fn create<'t>(
        context: Pin<&'t MutexContext>,
        name: &'static CStr,
        options: MutexOptions,
    ) -> Result<&'t Mutex, CreateError> {
        // Atomically claim the context to prevent race conditions.
        // compare_exchange ensures only one thread can successfully claim.
        if context
            .created
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Relaxed)
            .is_err()
        {
            return Err(CreateError::AlreadyCreated);
        }

        let inherit = if options.priority_inherit {
            tx_ffi::TX_INHERIT
        } else {
            tx_ffi::TX_NO_INHERIT
        };

        let result = unsafe {
            tx_ffi::_tx_mutex_create(context.mutex.as_ptr(), name.as_ptr() as *mut CHAR, inherit)
        };

        match result {
            tx_ffi::TX_SUCCESS => Ok(&context.get_ref().mutex),
            tx_ffi::TX_CALLER_ERROR => {
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

    /// Acquires the mutex, blocking indefinitely until available.
    ///
    /// Returns an RAII guard that releases the mutex when dropped.
    ///
    /// # Warning
    ///
    /// Do not call from ISR context - will return `InvalidCaller`.
    pub fn lock(&self) -> Result<MutexGuard<'_>, LockError> {
        self.lock_with_timeout(tx_ffi::TX_WAIT_FOREVER)
    }

    /// Attempts to acquire the mutex without blocking.
    ///
    /// Returns `Err(LockError::WouldBlock)` if the mutex is already held.
    pub fn try_lock(&self) -> Result<MutexGuard<'_>, LockError> {
        self.lock_with_timeout(tx_ffi::TX_NO_WAIT)
    }

    /// Attempts to acquire the mutex with a timeout.
    ///
    /// # Arguments
    ///
    /// - `timeout`: Maximum ticks to wait (TX_NO_WAIT for non-blocking,
    ///   TX_WAIT_FOREVER for indefinite)
    pub fn lock_with_timeout(&self, timeout: ULONG) -> Result<MutexGuard<'_>, LockError> {
        let result = unsafe { tx_ffi::_tx_mutex_get(self.as_ptr(), timeout) };

        match result {
            tx_ffi::TX_SUCCESS => Ok(MutexGuard {
                mutex: self,
                _not_send: core::marker::PhantomData,
            }),
            tx_ffi::TX_NOT_AVAILABLE => Err(LockError::WouldBlock),
            tx_ffi::TX_WAIT_ABORTED => Err(LockError::WaitAborted),
            tx_ffi::TX_MUTEX_ERROR => Err(LockError::Deleted),
            tx_ffi::TX_CALLER_ERROR => Err(LockError::InvalidCaller),
            _ => {
                // Check if it was a timeout (non-success with timeout specified)
                if timeout != tx_ffi::TX_WAIT_FOREVER && timeout != tx_ffi::TX_NO_WAIT {
                    Err(LockError::Timeout)
                } else {
                    Err(LockError::ThreadXError(result))
                }
            }
        }
    }

    /// Releases the mutex (internal, called by MutexGuard::drop).
    fn unlock(&self) -> Result<(), UINT> {
        let result = unsafe { tx_ffi::_tx_mutex_put(self.as_ptr()) };
        if result == tx_ffi::TX_SUCCESS {
            Ok(())
        } else {
            Err(result)
        }
    }

    /// Deletes the mutex, removing it from ThreadX.
    ///
    /// Any threads waiting on the mutex will receive an error.
    pub fn delete(&self) -> Result<(), UINT> {
        let result = unsafe { tx_ffi::_tx_mutex_delete(self.as_ptr()) };
        if result == tx_ffi::TX_SUCCESS {
            Ok(())
        } else {
            Err(result)
        }
    }

    /// Gets the mutex's name.
    ///
    /// # Safety
    ///
    /// This reads from the ThreadX control block.
    pub unsafe fn name(&self) -> Option<&'static str> {
        let name_ptr = (*self.as_ptr()).tx_mutex_name;
        if name_ptr.is_null() {
            return None;
        }
        let c_str = CStr::from_ptr(name_ptr);
        c_str.to_str().ok()
    }

    /// Returns the current owner thread (if locked).
    ///
    /// # Safety
    ///
    /// This reads from the ThreadX control block.
    pub unsafe fn owner(&self) -> Option<*mut tx_ffi::TX_THREAD> {
        let owner = (*self.as_ptr()).tx_mutex_owner;
        if owner.is_null() {
            None
        } else {
            Some(owner)
        }
    }

    /// Returns the current lock count (for recursive mutexes).
    ///
    /// # Safety
    ///
    /// This reads from the ThreadX control block.
    pub unsafe fn count(&self) -> ULONG {
        (*self.as_ptr()).tx_mutex_ownership_count
    }
}

/// RAII guard for a locked mutex.
///
/// When this guard is dropped, the mutex is automatically released.
/// This ensures mutexes are always properly unlocked, even on panic.
///
/// This type is `!Send` to prevent sending the guard to another thread,
/// since ThreadX mutexes are owned by the thread that locked them.
#[must_use = "if unused, the mutex will immediately unlock"]
pub struct MutexGuard<'a> {
    mutex: &'a Mutex,
    /// Marker to make MutexGuard !Send (raw pointers are !Send)
    _not_send: core::marker::PhantomData<*const ()>,
}

impl Drop for MutexGuard<'_> {
    fn drop(&mut self) {
        // Ignore error on unlock - we're in drop, not much we can do
        let _ = self.mutex.unlock();
    }
}

/// A mutex that protects data, similar to `std::sync::Mutex`.
///
/// This is a higher-level wrapper that combines a mutex with the data it protects,
/// providing safe access through RAII guards.
///
/// # Example
///
/// ```no_run
/// use cortex_r5_sample::safe::mutex::{DataMutex, MutexContext, MutexOptions};
/// use static_cell::StaticCell;
/// use core::pin::Pin;
///
/// // The protected data
/// struct SharedState {
///     counter: u32,
/// }
///
/// static MUTEX_CTX: StaticCell<MutexContext> = StaticCell::new(MutexContext::new());
/// static SHARED_DATA: StaticCell<SharedState> = StaticCell::new(SharedState { counter: 0 });
///
/// // After creating mutex:
/// // let mutex = Mutex::create(...);
/// // let data_mutex = DataMutex::new(mutex, &SHARED_DATA);
/// //
/// // In thread:
/// // let mut guard = data_mutex.lock().unwrap();
/// // guard.counter += 1;
/// ```
pub struct DataMutex<'a, T> {
    mutex: &'a Mutex,
    data: UnsafeCell<T>,
}

// DataMutex is Sync if T is Send (mutex protects access)
unsafe impl<T: Send> Sync for DataMutex<'_, T> {}

impl<'a, T> DataMutex<'a, T> {
    /// Creates a new DataMutex wrapping the given data.
    ///
    /// # Safety
    ///
    /// The mutex must be created and remain valid for the lifetime 'a.
    pub const fn new(mutex: &'a Mutex, data: T) -> Self {
        Self {
            mutex,
            data: UnsafeCell::new(data),
        }
    }

    /// Acquires the mutex and returns a guard for accessing the data.
    pub fn lock(&self) -> Result<DataGuard<'_, T>, LockError> {
        // Acquire the lock - the MutexGuard is stored in DataGuard to ensure
        // proper unlock on drop (fixes double-unlock bug)
        let guard = self.mutex.lock()?;
        Ok(DataGuard {
            _guard: guard,
            data: &self.data,
        })
    }

    /// Attempts to acquire the mutex without blocking.
    pub fn try_lock(&self) -> Result<DataGuard<'_, T>, LockError> {
        // Acquire the lock - the MutexGuard is stored in DataGuard to ensure
        // proper unlock on drop (fixes double-unlock bug)
        let guard = self.mutex.try_lock()?;
        Ok(DataGuard {
            _guard: guard,
            data: &self.data,
        })
    }
}

/// RAII guard for DataMutex that provides access to protected data.
///
/// This type is `!Send` because it contains a `MutexGuard` which is `!Send`.
/// The mutex is automatically released when this guard is dropped.
pub struct DataGuard<'a, T> {
    /// The underlying mutex guard - handles unlock on drop.
    /// Field order matters: _guard must be dropped after data access ends.
    _guard: MutexGuard<'a>,
    data: &'a UnsafeCell<T>,
}

impl<T> Deref for DataGuard<'_, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        // SAFETY: We hold the mutex lock via _guard
        unsafe { &*self.data.get() }
    }
}

impl<T> DerefMut for DataGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        // SAFETY: We hold the mutex lock exclusively via _guard
        unsafe { &mut *self.data.get() }
    }
}

// DataGuard no longer needs a custom Drop - MutexGuard handles unlock
