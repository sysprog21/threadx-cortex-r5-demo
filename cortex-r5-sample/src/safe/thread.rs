//! Type-safe ThreadX thread abstraction using Context + Resource pattern.
//!
//! This module provides safe wrappers around ThreadX thread primitives that separate
//! storage (Context) from handles (Resource), preventing common memory safety issues:
//!
//! - **Pinning**: Thread control blocks cannot move after creation
//! - **Lifetime tracking**: Handles cannot outlive their backing storage
//! - **Creation tracking**: Double-creation is prevented at runtime
//!
//! # Pattern Overview
//!
//! ```text
//! ThreadContext (owns TX_THREAD control block)
//!       │
//!       │ Thread::create(Pin<&ThreadContext>, ...)
//!       ▼
//! &Thread (safe handle for operations)
//! ```
//!
//! # Example
//!
//! ```no_run
//! use cortex_r5_sample::safe::thread::{Thread, ThreadContext, ThreadOptions};
//! use static_cell::StaticCell;
//! use core::pin::Pin;
//!
//! static THREAD_CTX: StaticCell<ThreadContext> = StaticCell::new();
//! static STACK: StaticCell<[u8; 4096]> = StaticCell::new();
//!
//! unsafe extern "C" fn my_entry(_: u32) {
//!     loop { unsafe { threadx_sys::_tx_thread_sleep(100); } }
//! }
//!
//! // In tx_application_define:
//! let ctx = unsafe { &*THREAD_CTX.uninit().assume_init_mut() };
//! let stack = unsafe { STACK.uninit().assume_init_mut() };
//! let pinned = unsafe { Pin::new_unchecked(ctx) };
//!
//! let thread = Thread::create(
//!     pinned,
//!     c"worker",
//!     stack,
//!     ThreadOptions {
//!         entry_fn: my_entry,
//!         priority: 10,
//!         ..Default::default()
//!     },
//! ).expect("thread create failed");
//!
//! // Later: access via context
//! // let thread = Pin::static_ref(&THREAD_CTX).try_created().unwrap();
//! ```

use core::cell::UnsafeCell;
use core::ffi::CStr;
use core::marker::PhantomPinned;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, Ordering};

use super::tx_ffi;
use super::TxError;
use tx_ffi::{CHAR, TX_THREAD, UINT, ULONG};

/// Errors that can occur during thread creation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CreateError {
    /// ThreadX returned an error code during creation
    ThreadXError(UINT),
    /// Stack pointer is null or invalid
    InvalidStack,
    /// Stack is not 8-byte aligned (required by ARM AAPCS and ThreadX)
    InvalidStackAlignment,
    /// Stack size is too small (minimum 1KB recommended)
    StackTooSmall,
    /// Priority value is invalid (must be 0-31)
    InvalidPriority,
    /// Thread already created with this context
    AlreadyCreated,
}

impl From<CreateError> for TxError {
    fn from(e: CreateError) -> Self {
        match e {
            CreateError::ThreadXError(code) => TxError::ThreadXError(code),
            CreateError::AlreadyCreated => TxError::AlreadyCreated,
            _ => TxError::InvalidParameter,
        }
    }
}

/// Thread priority value (0 = highest, 31 = lowest in most configurations).
pub type Priority = UINT;

/// Thread entry point function signature.
///
/// The entry parameter is passed from `ThreadOptions::entry_input`.
pub type EntryFn = unsafe extern "C" fn(entry: ULONG);

/// Options for configuring thread behavior.
#[derive(Clone, Copy)]
pub struct ThreadOptions {
    /// Thread entry point function
    pub entry_fn: EntryFn,
    /// Input parameter passed to entry function
    pub entry_input: ULONG,
    /// Thread priority (0 = highest, 31 = lowest)
    pub priority: Priority,
    /// Thread preemption threshold (must be ≤ priority)
    pub preemption_threshold: Priority,
    /// Time slice in ticks (0 = no time slicing)
    pub time_slice: ULONG,
    /// Auto-start thread after creation
    pub auto_start: bool,
}

impl Default for ThreadOptions {
    fn default() -> Self {
        Self {
            entry_fn: default_entry,
            entry_input: 0,
            priority: 16,
            preemption_threshold: 16,
            time_slice: tx_ffi::TX_NO_TIME_SLICE,
            auto_start: true,
        }
    }
}

/// Default entry point (sleeps forever).
unsafe extern "C" fn default_entry(_: ULONG) {
    loop {
        tx_ffi::_tx_thread_sleep(1000);
    }
}

/// Thread context storage (owns the ThreadX control block).
///
/// This structure contains the actual ThreadX control block and must remain
/// pinned after creation. Use `static` storage or pinned heap allocation.
///
/// # Pinning Requirement
///
/// ThreadX stores pointers to this control block in internal linked lists.
/// Moving it would corrupt those pointers, causing undefined behavior.
///
/// # Drop Behavior
///
/// **WARNING**: This type has no `Drop` implementation. If you drop a created
/// context without first terminating and deleting the thread, ThreadX will
/// retain dangling pointers, causing undefined behavior.
///
/// Always use `'static` storage for thread contexts, or ensure proper cleanup.
#[repr(C)]
pub struct ThreadContext {
    /// The wrapped Thread resource (provides API)
    thread: Thread,
    /// Tracks whether the thread has been created
    created: AtomicBool,
}

// ThreadContext is Sync because we use AtomicBool and Thread is Sync.
unsafe impl Sync for ThreadContext {}

impl ThreadContext {
    /// Creates a new thread context in uninitialized state.
    ///
    /// The returned context must be pinned before calling `Thread::create()`.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use static_cell::StaticCell;
    /// use cortex_r5_sample::safe::thread::ThreadContext;
    ///
    /// static THREAD_CTX: StaticCell<ThreadContext> = StaticCell::new();
    /// ```
    pub const fn new() -> Self {
        Self {
            thread: Thread::new(),
            created: AtomicBool::new(false),
        }
    }

    /// Resets the created flag after thread deletion.
    ///
    /// # Safety
    ///
    /// Only call after `Thread::delete()` succeeds. Calling while a thread
    /// is registered with ThreadX allows double-creation, causing UB.
    pub unsafe fn reset_created(&self) {
        self.created.store(false, Ordering::Release);
    }

    /// Returns `true` if this thread has been created.
    #[inline]
    pub fn is_created(&self) -> bool {
        self.created.load(Ordering::Acquire)
    }

    /// Returns the thread handle if it has been created.
    ///
    /// This is useful when you need to access a globally-allocated thread
    /// from a different scope than where it was created.
    #[inline]
    pub fn try_created(self: Pin<&Self>) -> Option<&Thread> {
        if self.is_created() {
            // SAFETY: We verified the thread is created
            Some(unsafe { self.assume_created() })
        } else {
            None
        }
    }

    /// Returns the thread handle without checking creation status.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `Thread::create()` was called and succeeded.
    #[inline]
    pub unsafe fn assume_created(self: Pin<&Self>) -> &Thread {
        &self.get_ref().thread
    }
}

/// Thread resource handle (safe API for thread operations).
///
/// This handle provides safe access to ThreadX thread operations. It borrows
/// a pointer to the control block owned by `ThreadContext`.
///
/// # Lifetime
///
/// The handle borrows from a pinned `ThreadContext`, ensuring it cannot
/// outlive the backing storage.
#[repr(transparent)]
pub struct Thread {
    /// ThreadX control block (interior mutability for FFI)
    control_block: UnsafeCell<TX_THREAD>,
    /// Prevents moving (ThreadX stores pointers to this)
    _pin: PhantomPinned,
}

// Thread is Sync because ThreadX APIs are thread-safe (use critical sections internally)
unsafe impl Sync for Thread {}

impl Thread {
    /// Creates a zeroed Thread (internal use only).
    const fn new() -> Self {
        Self {
            // SAFETY: Zero is valid for TX_THREAD (all-zero initialization)
            control_block: UnsafeCell::new(unsafe { core::mem::zeroed() }),
            _pin: PhantomPinned,
        }
    }

    /// Get raw pointer to control block (for FFI calls).
    #[inline]
    fn as_ptr(&self) -> *mut TX_THREAD {
        self.control_block.get()
    }

    /// Creates and initializes a ThreadX thread.
    ///
    /// # Arguments
    ///
    /// - `context`: Pinned context storage (immutable reference suffices)
    /// - `name`: Null-terminated thread name (must have 'static lifetime)
    /// - `stack`: Stack memory (must be 8-byte aligned, minimum 1KB)
    /// - `options`: Thread configuration
    ///
    /// # Returns
    ///
    /// A reference to the `Thread` resource tied to the context's lifetime.
    /// This reference is valid as long as the pinned context exists.
    ///
    /// # Errors
    ///
    /// Returns `CreateError` if:
    /// - Thread already created with this context
    /// - Stack is misaligned or too small
    /// - Priority values are invalid
    /// - ThreadX creation fails
    ///
    /// # Example
    ///
    /// ```no_run
    /// use cortex_r5_sample::safe::thread::{Thread, ThreadContext, ThreadOptions};
    /// use static_cell::StaticCell;
    /// use core::pin::Pin;
    ///
    /// static THREAD_CTX: StaticCell<ThreadContext> = StaticCell::new();
    /// static STACK: StaticCell<[u8; 4096]> = StaticCell::new();
    ///
    /// unsafe extern "C" fn entry(_: u32) {
    ///     loop { unsafe { threadx_sys::_tx_thread_sleep(100); } }
    /// }
    ///
    /// let ctx = unsafe { &*THREAD_CTX.uninit().assume_init_mut() };
    /// let stack = unsafe { STACK.uninit().assume_init_mut() };
    /// let pinned = unsafe { Pin::new_unchecked(ctx) };
    ///
    /// let thread = Thread::create(
    ///     pinned,
    ///     c"worker",
    ///     stack,
    ///     ThreadOptions { entry_fn: entry, priority: 10, ..Default::default() },
    /// ).unwrap();
    /// ```
    pub fn create<'t>(
        context: Pin<&'t ThreadContext>,
        name: &'static CStr,
        stack: &'static mut [u8],
        options: ThreadOptions,
    ) -> Result<&'t Thread, CreateError> {
        // Atomically claim the context to prevent race conditions.
        // compare_exchange ensures only one thread can successfully claim.
        if context
            .created
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Relaxed)
            .is_err()
        {
            return Err(CreateError::AlreadyCreated);
        }

        // Validate stack
        if stack.is_empty() {
            context.created.store(false, Ordering::Release);
            return Err(CreateError::InvalidStack);
        }

        const MIN_STACK_SIZE: usize = 1024;
        if stack.len() < MIN_STACK_SIZE {
            context.created.store(false, Ordering::Release);
            return Err(CreateError::StackTooSmall);
        }

        // ARM AAPCS requires 8-byte stack alignment
        let stack_ptr = stack.as_ptr() as usize;
        if stack_ptr % 8 != 0 || stack.len() % 8 != 0 {
            context.created.store(false, Ordering::Release);
            return Err(CreateError::InvalidStackAlignment);
        }

        // Validate priority (0-31 for standard ThreadX)
        if options.priority > 31 {
            context.created.store(false, Ordering::Release);
            return Err(CreateError::InvalidPriority);
        }

        // Preemption threshold must be ≤ priority
        if options.preemption_threshold > options.priority {
            context.created.store(false, Ordering::Release);
            return Err(CreateError::InvalidPriority);
        }

        let auto_start = if options.auto_start {
            tx_ffi::TX_AUTO_START
        } else {
            tx_ffi::TX_DONT_START
        };

        // SAFETY: We have a pinned reference to an uninitialized context.
        // The context is pinned, so the control block won't move.
        // ThreadX will initialize the control block.
        let result = unsafe {
            tx_ffi::_tx_thread_create(
                context.thread.as_ptr(),
                name.as_ptr() as *mut CHAR,
                Some(options.entry_fn),
                options.entry_input,
                stack.as_mut_ptr() as *mut core::ffi::c_void,
                stack.len() as ULONG,
                options.priority,
                options.preemption_threshold,
                options.time_slice,
                auto_start,
            )
        };

        if result != tx_ffi::TX_SUCCESS {
            // Roll back - allow re-creation attempts
            context.created.store(false, Ordering::Release);
            return Err(CreateError::ThreadXError(result));
        }

        // Return reference to the Thread within the context
        // SAFETY: The context is pinned, so the Thread reference is stable.
        // The lifetime 't ties the reference to the pinned context.
        Ok(&context.get_ref().thread)
    }

    /// Suspends the thread.
    ///
    /// The thread will not execute until `resume()` is called.
    ///
    /// # Warning
    ///
    /// Suspending a thread while it holds resources (mutexes, semaphores)
    /// may cause deadlocks.
    pub fn suspend(&self) -> Result<(), UINT> {
        let result = unsafe { tx_ffi::_tx_thread_suspend(self.as_ptr()) };
        if result == tx_ffi::TX_SUCCESS {
            Ok(())
        } else {
            Err(result)
        }
    }

    /// Resumes a suspended thread.
    pub fn resume(&self) -> Result<(), UINT> {
        let result = unsafe { tx_ffi::_tx_thread_resume(self.as_ptr()) };
        if result == tx_ffi::TX_SUCCESS {
            Ok(())
        } else {
            Err(result)
        }
    }

    /// Terminates the thread.
    ///
    /// The thread must be terminated before deletion.
    pub fn terminate(&self) -> Result<(), UINT> {
        let result = unsafe { tx_ffi::_tx_thread_terminate(self.as_ptr()) };
        if result == tx_ffi::TX_SUCCESS {
            Ok(())
        } else {
            Err(result)
        }
    }

    /// Resets a terminated thread to ready state.
    pub fn reset(&self) -> Result<(), UINT> {
        let result = unsafe { tx_ffi::_tx_thread_reset(self.as_ptr()) };
        if result == tx_ffi::TX_SUCCESS {
            Ok(())
        } else {
            Err(result)
        }
    }

    /// Deletes the thread, removing it from ThreadX.
    ///
    /// The thread must be terminated first.
    ///
    /// # Safety
    ///
    /// After deletion, the context can be reused by calling
    /// `ThreadContext::reset_created()` followed by `Thread::create()`.
    pub fn delete(&self) -> Result<(), UINT> {
        let result = unsafe { tx_ffi::_tx_thread_delete(self.as_ptr()) };
        if result == tx_ffi::TX_SUCCESS {
            Ok(())
        } else {
            Err(result)
        }
    }

    /// Gets the thread's name.
    ///
    /// # Safety
    ///
    /// This reads from the ThreadX control block which may be concurrently
    /// modified by the scheduler. Safe to call when thread is suspended.
    pub unsafe fn name(&self) -> Option<&'static str> {
        let name_ptr = (*self.as_ptr()).tx_thread_name;
        if name_ptr.is_null() {
            return None;
        }
        let c_str = CStr::from_ptr(name_ptr);
        c_str.to_str().ok()
    }

    /// Gets the thread's current priority.
    ///
    /// # Safety
    ///
    /// This reads from the ThreadX control block. Safe when thread is suspended.
    pub unsafe fn priority(&self) -> Priority {
        (*self.as_ptr()).tx_thread_priority
    }

    /// Gets the thread's state.
    ///
    /// # Safety
    ///
    /// This reads from the ThreadX control block. The state may change
    /// immediately after this call returns.
    pub unsafe fn state(&self) -> UINT {
        (*self.as_ptr()).tx_thread_state
    }
}

/// Sleep the current thread for the specified number of ticks.
///
/// # Safety
///
/// Must be called from a thread context (not ISR or initialization).
#[inline]
pub fn sleep(ticks: ULONG) {
    unsafe {
        tx_ffi::_tx_thread_sleep(ticks);
    }
}

/// Relinquish control to other threads at the same priority.
#[inline]
pub fn relinquish() {
    unsafe {
        tx_ffi::_tx_thread_relinquish();
    }
}

/// Identify the currently executing thread.
///
/// Returns `None` if called before kernel initialization or from ISR
/// during initialization.
pub fn identify() -> Option<*mut TX_THREAD> {
    let ptr = unsafe { tx_ffi::_tx_thread_identify() };
    if ptr.is_null() {
        None
    } else {
        Some(ptr)
    }
}
