//! Verify that Queue handles cannot outlive their QueueContext.
//!
//! The Queue reference's lifetime is tied to the pinned QueueContext,
//! preventing dangling pointers to the control block.

use cortex_r5_sample::safe::queue::{queue_storage_size, Queue, QueueContext, QueueOptions};
use std::pin::pin;

static mut STORAGE: [u32; queue_storage_size::<u32>(16)] = [0; queue_storage_size::<u32>(16)];

fn main() {
    let queue: &Queue<u32> = {
        let ctx = pin!(QueueContext::<u32>::new());
        let storage = unsafe { &mut STORAGE };
        Queue::create(ctx.as_ref(), c"test", storage, QueueOptions::default()).unwrap()
        // ctx is dropped here
    };

    // This should fail: queue handle cannot outlive context
    let _ = queue.try_send(&42u32);
}
