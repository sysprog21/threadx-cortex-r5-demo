//! Verify that DataGuard cannot be sent to another thread.
//!
//! DataGuard wraps a MutexGuard which is !Send, so DataGuard
//! should also be !Send.

use cortex_r5_sample::safe::mutex::{DataMutex, Mutex, MutexContext, MutexOptions};
use std::pin::pin;

/// Helper function that requires T: Send to test the bound.
fn requires_send<T: Send>(_: T) {}

fn main() {
    let ctx = pin!(MutexContext::new());
    let mutex = Mutex::create(ctx.as_ref(), c"test", MutexOptions::default()).unwrap();
    let data_mutex = DataMutex::new(mutex, 42u32);
    let guard = data_mutex.lock().unwrap();

    // This should fail: DataGuard is !Send because MutexGuard is !Send
    requires_send(guard);
}
