//! Verify that MutexGuard cannot be sent to another thread.
//!
//! MutexGuard contains `PhantomData<*const ()>` which makes it !Send.
//! This is critical because ThreadX mutex ownership is thread-specific.

use cortex_r5_sample::safe::mutex::{Mutex, MutexContext, MutexOptions};
use std::pin::pin;

/// Helper function that requires T: Send to test the bound.
fn requires_send<T: Send>(_: T) {}

fn main() {
    let ctx = pin!(MutexContext::new());
    let mutex = Mutex::create(ctx.as_ref(), c"test", MutexOptions::default()).unwrap();
    let guard = mutex.lock().unwrap();

    // This should fail: MutexGuard is !Send
    requires_send(guard);
}
