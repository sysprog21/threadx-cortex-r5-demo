//! Verify that Thread handles cannot outlive their ThreadContext.
//!
//! The Thread reference's lifetime is tied to the pinned ThreadContext,
//! preventing dangling pointers to the control block.

use cortex_r5_sample::safe::thread::{Thread, ThreadContext, ThreadOptions};
use std::pin::pin;

static mut STACK: [u8; 4096] = [0u8; 4096];

unsafe extern "C" fn dummy_entry(_: u32) {
    loop {}
}

fn main() {
    let thread: &Thread = {
        let ctx = pin!(ThreadContext::new());
        let stack = unsafe { &mut STACK };
        Thread::create(
            ctx.as_ref(),
            c"test",
            stack,
            ThreadOptions {
                entry_fn: dummy_entry,
                priority: 10,
                ..Default::default()
            },
        )
        .unwrap()
        // ctx is dropped here
    };

    // This should fail: thread handle cannot outlive context
    let _ = thread.suspend();
}
