//! Verify that MutexGuard cannot outlive the Mutex it was obtained from.
//!
//! The guard's lifetime is tied to the mutex reference, preventing
//! use-after-free of the underlying control block.

use cortex_r5_sample::safe::mutex::{Mutex, MutexContext, MutexGuard, MutexOptions};
use std::pin::pin;

fn main() {
    let guard: MutexGuard<'_> = {
        let ctx = pin!(MutexContext::new());
        let mutex = Mutex::create(ctx.as_ref(), c"test", MutexOptions::default()).unwrap();
        mutex.lock().unwrap()
        // mutex and ctx are dropped here
    };

    // This should fail: guard cannot outlive mutex
    drop(guard);
}
