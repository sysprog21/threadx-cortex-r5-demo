//! Type-safe ThreadX thread abstraction with pinning guarantees.
//!
//! This module provides safe wrappers around ThreadX thread primitives that prevent
//! common memory safety issues through Rust's type system:
//!
//! - **Pinning**: Thread control blocks cannot move after creation (prevents linked list corruption)
//! - **Lifetime tracking**: Stack borrows are tracked via `PhantomData` (prevents use-after-free)
//! - **Context + Resource pattern**: Storage (Context) is separate from handle (Thread)
//!
//! # Safety Guarantees
//!
//! ThreadX maintains internal linked lists (`_tx_thread_created_ptr`) that store pointers
//! to thread control blocks. If a control block moves in memory after registration,
//! these pointers become dangling, causing undefined behavior during:
//! - Thread enumeration
//! - Scheduler operations
//! - Debug/trace operations
//!
//! The `PhantomPinned` marker on `ThreadContext` prevents moving via Rust's `!Unpin` trait,
//! ensuring control blocks remain at a fixed memory location after creation.

use core::marker::{PhantomData, PhantomPinned};
use core::pin::Pin;
use threadx_sys::{CHAR, TX_THREAD, UINT, ULONG};

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
    /// Priority value is invalid (must be 1-31, 0 reserved for system)
    InvalidPriority,
    /// Thread already created with this context (call delete first)
    AlreadyCreated,
}

/// Thread priority value (1 = highest, 31 = lowest in most configurations).
///
/// ThreadX supports priority values from 0-31, where lower numbers indicate
/// higher priority. Priority 0 is typically reserved for system threads.
pub type Priority = UINT;

/// Thread entry point function signature.
///
/// The entry parameter is passed from `ThreadOptions::entry_input` and can
/// be used to pass context or configuration to the thread.
pub type EntryFn = unsafe extern "C" fn(entry: ULONG);

/// Options for configuring thread behavior.
#[derive(Clone, Copy)]
pub struct ThreadOptions {
    /// Thread entry point function
    pub entry_fn: EntryFn,
    /// Input parameter passed to entry function
    pub entry_input: ULONG,
    /// Thread priority (1 = highest, 31 = lowest)
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
            time_slice: threadx_sys::TX_NO_TIME_SLICE,
            auto_start: true,
        }
    }
}

/// Default entry point (does nothing, for testing).
unsafe extern "C" fn default_entry(_: ULONG) {
    loop {
        threadx_sys::_tx_thread_sleep(1000);
    }
}

/// Thread context storage (owns the ThreadX control block).
///
/// This structure contains the actual ThreadX control block and cannot be moved
/// after the thread is created. It must be stored in a static location (e.g., via
/// `static_cell::StaticCell` or pinned on the heap).
///
/// # Lifetime Parameters
///
/// - `'stack`: Lifetime of the borrowed stack memory
///
/// # Pinning Requirement
///
/// The `PhantomPinned` marker prevents this structure from being moved after creation.
/// ThreadX stores a pointer to the control block in its internal linked lists, so
/// moving the structure would invalidate these pointers.
///
/// # Drop Safety
///
/// **CRITICAL**: `ThreadContext` has no `Drop` implementation. If a thread is created
/// and the context is dropped (e.g., by returning from a function with stack-allocated
/// context), ThreadX will retain a dangling pointer in its global linked lists, causing
/// undefined behavior when the scheduler runs.
///
/// **Safety Requirements**:
/// - Use `'static` storage (via `static` or `StaticCell`) for thread contexts
/// - OR ensure the thread is terminated and deleted before dropping the context
/// - OR use a scoped thread pattern that guarantees proper cleanup
///
/// **Example (Safe - Static Storage)**:
/// ```no_run
/// static mut THREAD_CTX: ThreadContext<'static> = unsafe { core::mem::zeroed() };
/// // Context lives for entire program lifetime, safe to register with ThreadX
/// ```
///
/// **Example (Unsafe - Stack Allocation)**:
/// ```no_run
/// fn create_thread() {
///     let mut ctx = ThreadContext::new();  // ⚠️ DANGER: Stack allocation
///     // ... create thread with Pin::new_unchecked(&mut ctx) ...
/// }  // ⚠️ ctx dropped here, ThreadX has dangling pointer -> UNDEFINED BEHAVIOR
/// ```
pub struct ThreadContext<'stack> {
    /// ThreadX control block (must not move after creation)
    control_block: TX_THREAD,
    /// Tracks whether the thread has been created (prevents double-init)
    created: core::cell::Cell<bool>,
    /// Tracks the borrowed stack lifetime
    _borrow: PhantomData<&'stack mut [u8]>,
    /// Prevents moving (implements !Unpin)
    _pin: PhantomPinned,
}

impl<'stack> ThreadContext<'stack> {
    /// Creates a new thread context in uninitialized state.
    ///
    /// This only allocates the context structure; you must call `Thread::create()`
    /// to initialize it and register with ThreadX.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use static_cell::StaticCell;
    /// use cortex_r5_sample::thread::ThreadContext;
    ///
    /// static THREAD_CTX: StaticCell<ThreadContext> = StaticCell::new(ThreadContext::new());
    /// ```
    pub const fn new() -> Self {
        Self {
            control_block: unsafe { core::mem::zeroed() },
            created: core::cell::Cell::new(false),
            _borrow: PhantomData,
            _pin: PhantomPinned,
        }
    }

    /// Checks if a thread has been created with this context.
    ///
    /// Returns `true` if `Thread::create()` was called and succeeded.
    /// Returns `false` if the context is uninitialized or after deletion.
    pub fn is_created(&self) -> bool {
        self.created.get()
    }

    /// Resets the created flag after thread deletion.
    ///
    /// # Safety
    ///
    /// This should only be called after `Thread::delete()` succeeds. Calling this
    /// while a thread is still registered with ThreadX will allow double-creation,
    /// leading to undefined behavior.
    ///
    /// # Example
    ///
    /// ```no_run
    /// // After deleting the thread:
    /// thread.delete().expect("Failed to delete thread");
    /// unsafe { context.reset_created(); }
    /// // Now the context can be reused
    /// ```
    pub unsafe fn reset_created(&self) {
        self.created.set(false);
    }
}

/// Thread resource handle (safe API for interacting with a created thread).
///
/// This handle provides safe access to ThreadX thread operations. It does not
/// own the control block (that's owned by `ThreadContext`), but holds a pointer
/// to it for API calls.
///
/// # Lifetime
///
/// The lifetime parameter `'a` ties this handle to the `ThreadContext` it was
/// created from, ensuring the handle cannot outlive the storage.
#[derive(Clone, Copy)]
pub struct Thread<'a> {
    /// Pointer to the thread control block (owned by ThreadContext)
    control_block: *mut TX_THREAD,
    /// Ties handle lifetime to context lifetime
    _marker: PhantomData<&'a ThreadContext<'a>>,
}

impl<'a> Thread<'a> {
    /// Creates and initializes a ThreadX thread.
    ///
    /// # Arguments
    ///
    /// - `context`: Pinned context storage (must remain pinned for thread's lifetime)
    /// - `name`: Null-terminated thread name (must have 'static lifetime)
    /// - `stack`: Borrowed stack memory (must outlive the thread)
    /// - `options`: Thread configuration options
    ///
    /// # Safety
    ///
    /// - `context` must remain pinned and valid for the thread's entire lifetime
    /// - `name` must be a valid null-terminated string with 'static lifetime
    /// - `stack` must be valid memory for the duration of `'stack`
    /// - Stack must be 8-byte aligned and sufficiently sized (minimum 1KB recommended)
    ///
    /// # Errors
    ///
    /// Returns `CreateError` if:
    /// - ThreadX creation fails (e.g., invalid priority, out of resources)
    /// - Stack is empty or not 8-byte aligned
    /// - Priority values are invalid
    ///
    /// # Example
    ///
    /// ```no_run
    /// use core::pin::Pin;
    /// use static_cell::StaticCell;
    /// use cortex_r5_sample::thread::{Thread, ThreadContext, ThreadOptions};
    ///
    /// static THREAD_CTX: StaticCell<ThreadContext> = StaticCell::new(ThreadContext::new());
    /// static STACK: StaticCell<[u8; 4096]> = StaticCell::new([0u8; 4096]);
    ///
    /// unsafe extern "C" fn my_entry(_: u32) {
    ///     loop {
    ///         threadx_sys::_tx_thread_sleep(100);
    ///     }
    /// }
    ///
    /// let thread = Thread::create(
    ///     Pin::new_unchecked(&mut THREAD_CTX),
    ///     c_str!("my-thread"),
    ///     &mut STACK,
    ///     ThreadOptions {
    ///         entry_fn: my_entry,
    ///         priority: 10,
    ///         ..Default::default()
    ///     },
    /// ).unwrap();
    /// ```
    pub fn create(
        mut context: Pin<&'a mut ThreadContext<'a>>,
        name: &'static core::ffi::CStr,
        stack: &'a mut [u8],
        options: ThreadOptions,
    ) -> Result<Self, CreateError> {
        // Check if thread already created with this context
        let ctx_ref = context.as_ref().get_ref();
        if ctx_ref.created.get() {
            return Err(CreateError::AlreadyCreated);
        }

        // Validate stack
        if stack.is_empty() {
            return Err(CreateError::InvalidStack);
        }

        // Minimum stack size (1KB) - ThreadX requires enough for context frame
        const MIN_STACK_SIZE: usize = 1024;
        if stack.len() < MIN_STACK_SIZE {
            return Err(CreateError::StackTooSmall);
        }

        // Check 8-byte alignment of base pointer (required by ARM AAPCS)
        let stack_ptr = stack.as_ptr() as usize;
        if stack_ptr % 8 != 0 {
            return Err(CreateError::InvalidStackAlignment);
        }

        // Check stack length is 8-byte aligned (for top-of-stack calculations)
        if stack.len() % 8 != 0 {
            return Err(CreateError::InvalidStackAlignment);
        }

        // Validate priority: 1-31 (0 reserved for system threads)
        // Note: Some ports may further restrict the range
        if options.priority == 0 || options.priority > 31 {
            return Err(CreateError::InvalidPriority);
        }

        // Validate preemption threshold: must be <= priority and within range
        if options.preemption_threshold == 0
            || options.preemption_threshold > 31
            || options.preemption_threshold > options.priority
        {
            return Err(CreateError::InvalidPriority);
        }

        // Safety: We have exclusive mutable access via Pin, and we're about to
        // initialize the control block. The pinning guarantee ensures it won't move.
        let control_block_ptr = unsafe {
            let ctx = context.as_mut().get_unchecked_mut();
            &mut ctx.control_block as *mut TX_THREAD
        };

        let auto_start = if options.auto_start {
            threadx_sys::TX_AUTO_START
        } else {
            threadx_sys::TX_DONT_START
        };

        // Create the ThreadX thread
        let result = unsafe {
            threadx_sys::_tx_thread_create(
                control_block_ptr,
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

        if result != threadx_sys::TX_SUCCESS {
            return Err(CreateError::ThreadXError(result));
        }

        // Mark context as created to prevent double-init
        unsafe {
            context.as_mut().get_unchecked_mut().created.set(true);
        }

        // Return thread handle by value.
        // The lifetime 'a ties the handle to the ThreadContext, ensuring the handle
        // cannot outlive the storage. This is safe because:
        // 1. control_block_ptr remains valid for 'a (pinned in context)
        // 2. Thread is Copy and just wraps a pointer
        // 3. PhantomData ties the lifetime to prevent use-after-free
        Ok(Thread {
            control_block: control_block_ptr,
            _marker: PhantomData,
        })
    }

    /// Deletes the thread and releases its resources.
    ///
    /// The thread must be in a terminated or completed state before deletion.
    /// After deletion, the `ThreadContext` can be reused to create a new thread.
    ///
    /// # Safety
    ///
    /// The thread must be terminated first (via `terminate()` or thread completion).
    /// Deleting a running thread causes undefined behavior in ThreadX.
    ///
    /// # Errors
    ///
    /// Returns the ThreadX error code if deletion fails (e.g., thread not terminated).
    ///
    /// # Note
    ///
    /// This method consumes the Thread handle. To reuse the ThreadContext, you must
    /// manually clear its `created` flag via `ThreadContext::reset_created()`.
    pub fn delete(self) -> Result<(), UINT> {
        let result = unsafe { threadx_sys::_tx_thread_delete(self.control_block) };
        if result == threadx_sys::TX_SUCCESS {
            // Note: We cannot clear the context's `created` flag here since we don't
            // have a reference to it. The caller must use ThreadContext::reset_created().
            Ok(())
        } else {
            Err(result)
        }
    }

    /// Suspends the thread.
    ///
    /// The thread will not execute until `resume()` is called.
    ///
    /// # Safety
    ///
    /// This function is safe to call, but suspending a thread while it holds
    /// resources (mutexes, semaphores) may cause deadlocks.
    pub fn suspend(&self) -> Result<(), UINT> {
        let result = unsafe { threadx_sys::_tx_thread_suspend(self.control_block) };
        if result == threadx_sys::TX_SUCCESS {
            Ok(())
        } else {
            Err(result)
        }
    }

    /// Resumes a suspended thread.
    ///
    /// If the thread was not suspended, this has no effect.
    pub fn resume(&self) -> Result<(), UINT> {
        let result = unsafe { threadx_sys::_tx_thread_resume(self.control_block) };
        if result == threadx_sys::TX_SUCCESS {
            Ok(())
        } else {
            Err(result)
        }
    }

    /// Terminates the thread.
    ///
    /// The thread's resources are released but the control block remains valid.
    /// The thread can be restarted with `reset()` followed by `resume()`.
    ///
    /// # Safety
    ///
    /// Terminating a thread while it holds resources may cause resource leaks.
    /// Ensure all resources are properly released before terminating.
    pub fn terminate(&self) -> Result<(), UINT> {
        let result = unsafe { threadx_sys::_tx_thread_terminate(self.control_block) };
        if result == threadx_sys::TX_SUCCESS {
            Ok(())
        } else {
            Err(result)
        }
    }

    /// Resets a terminated or completed thread to the ready state.
    ///
    /// The thread must be terminated first before calling this.
    pub fn reset(&self) -> Result<(), UINT> {
        let result = unsafe { threadx_sys::_tx_thread_reset(self.control_block) };
        if result == threadx_sys::TX_SUCCESS {
            Ok(())
        } else {
            Err(result)
        }
    }

    /// Gets the thread's name.
    ///
    /// Returns `None` if the name pointer is null or invalid.
    ///
    /// # Safety
    ///
    /// **Data Race Warning**: This method reads from ThreadX control block fields that
    /// may be concurrently modified by the scheduler or ISR (preemption = concurrency).
    /// This is technically a data race under Rust's memory model, even on single-core.
    ///
    /// **Safe Usage Patterns**:
    /// - Call from within a ThreadX critical section (interrupts disabled)
    /// - Call only when the thread is suspended or not yet started
    /// - Use `tx_thread_info_get` API for synchronized reads (if available)
    ///
    /// The returned string slice has 'static lifetime because ThreadX requires
    /// thread names to be static strings (they're stored as pointers, not copied).
    pub unsafe fn name(&self) -> Option<&'static str> {
        let name_ptr = (*self.control_block).tx_thread_name;
        if name_ptr.is_null() {
            return None;
        }

        let c_str = core::ffi::CStr::from_ptr(name_ptr);
        c_str.to_str().ok()
    }

    /// Gets the thread's current priority.
    ///
    /// # Safety
    ///
    /// **Data Race Warning**: Reads from the ThreadX control block which may be
    /// concurrently modified by the scheduler. This can observe torn reads on
    /// architectures without atomic 32-bit loads, or stale values due to preemption.
    ///
    /// **Safe Usage**: Call from within a critical section or while the thread is suspended.
    pub unsafe fn priority(&self) -> Priority {
        (*self.control_block).tx_thread_priority
    }

    /// Gets the thread's state.
    ///
    /// Returns the raw ThreadX state value:
    /// - 0: Ready
    /// - 1: Completed
    /// - 2: Terminated
    /// - 3: Suspended
    /// - 4: Sleep
    /// - 5: Queue suspended
    /// - 6: Semaphore suspended
    /// - 7: Event flag suspended
    /// - 8: Memory suspended
    /// - 9: Mutex suspended
    ///
    /// # Safety
    ///
    /// **Data Race Warning**: Reads from the ThreadX control block which may be
    /// concurrently modified by the scheduler. The state can change immediately after
    /// this call returns, even within a critical section (thread state transitions
    /// can occur when interrupts are re-enabled).
    ///
    /// **Safe Usage**: For diagnostic/debugging only. Do not use for control flow
    /// decisions unless the thread is suspended.
    pub unsafe fn state(&self) -> UINT {
        (*self.control_block).tx_thread_state
    }
}

// Thread handles are safe to send between threads (ThreadX APIs are thread-safe)
unsafe impl<'a> Send for Thread<'a> {}

// Thread handles can be shared between threads because:
// 1. ThreadX APIs (suspend, resume, etc.) are thread-safe (disable interrupts internally)
// 2. Thread is Copy, so users can make independent handles anyway
// 3. The ThreadX control block is managed by ThreadX, not Rust
unsafe impl<'a> Sync for Thread<'a> {}
