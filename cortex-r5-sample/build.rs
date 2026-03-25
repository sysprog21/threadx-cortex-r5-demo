//! Build script for the Rust and ThreadX demonstration.

use std::{env, error::Error, path::PathBuf};

static TX_PORT_FILES: &[&str] = &[
    "tx_thread_context_restore.S",
    "tx_thread_fiq_nesting_end.S",
    "tx_thread_interrupt_restore.S",
    "tx_thread_stack_build.S",
    "tx_thread_context_save.S",
    "tx_thread_fiq_nesting_start.S",
    "tx_thread_irq_nesting_end.S",
    "tx_thread_system_return.S",
    "tx_thread_fiq_context_restore.S",
    "tx_thread_interrupt_control.S",
    "tx_thread_irq_nesting_start.S",
    "tx_thread_vectored_context_save.S",
    "tx_thread_fiq_context_save.S",
    "tx_thread_interrupt_disable.S",
    "tx_thread_schedule.S",
    "tx_timer_interrupt.S",
];

static TX_COMMON_FILES: &[&str] = &[
    // Byte Pool (Used for stacks)
    "tx_byte_allocate.c",
    "tx_byte_pool_cleanup.c",
    "tx_byte_pool_create.c",
    "tx_byte_pool_delete.c",
    "tx_byte_pool_info_get.c",
    "tx_byte_pool_initialize.c",
    "tx_byte_pool_prioritize.c",
    "tx_byte_pool_search.c",
    "tx_byte_release.c",
    // Event Flags
    "tx_event_flags_cleanup.c",
    "tx_event_flags_create.c",
    "tx_event_flags_delete.c",
    "tx_event_flags_get.c",
    "tx_event_flags_info_get.c",
    "tx_event_flags_initialize.c",
    "tx_event_flags_set.c",
    "tx_event_flags_set_notify.c",
    // Initialization
    "tx_initialize_high_level.c",
    "tx_initialize_kernel_enter.c",
    "tx_initialize_kernel_setup.c",
    // Mutex
    "tx_mutex_cleanup.c",
    "tx_mutex_create.c",
    "tx_mutex_delete.c",
    "tx_mutex_get.c",
    "tx_mutex_info_get.c",
    "tx_mutex_initialize.c",
    "tx_mutex_prioritize.c",
    "tx_mutex_priority_change.c",
    "tx_mutex_put.c",
    // Queue
    "tx_queue_cleanup.c",
    "tx_queue_create.c",
    "tx_queue_delete.c",
    "tx_queue_flush.c",
    "tx_queue_front_send.c",
    "tx_queue_info_get.c",
    "tx_queue_initialize.c",
    "tx_queue_prioritize.c",
    "tx_queue_receive.c",
    "tx_queue_send.c",
    "tx_queue_send_notify.c",
    // Thread
    "tx_thread_create.c",
    "tx_thread_delete.c",
    "tx_thread_entry_exit_notify.c",
    "tx_thread_identify.c",
    "tx_thread_info_get.c",
    "tx_thread_initialize.c",
    "tx_thread_preemption_change.c",
    "tx_thread_priority_change.c",
    "tx_thread_relinquish.c",
    "tx_thread_reset.c",
    "tx_thread_resume.c",
    "tx_thread_shell_entry.c",
    "tx_thread_sleep.c",
    "tx_thread_suspend.c",
    "tx_thread_system_preempt_check.c",
    "tx_thread_system_resume.c",
    "tx_thread_system_suspend.c",
    "tx_thread_terminate.c",
    "tx_thread_time_slice.c",
    "tx_thread_time_slice_change.c",
    "tx_thread_timeout.c",
    "tx_thread_wait_abort.c",
    // Time/Timer (System only, no software timers)
    "tx_time_get.c",
    "tx_time_set.c",
    "tx_timer_expiration_process.c",
    "tx_timer_initialize.c",
    "tx_timer_system_activate.c",
    "tx_timer_system_deactivate.c",
    "tx_timer_thread_entry.c",
];

fn main() -> Result<(), Box<dyn Error>> {
    let crate_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR")?);

    // Skip ThreadX compilation when host-test feature is enabled.
    // This allows compile-fail tests to run on the host platform.
    if env::var("CARGO_FEATURE_HOST_TEST").is_ok() {
        println!("cargo:warning=host-test feature enabled, skipping ThreadX compilation");
        return Ok(());
    }

    let has_versatileab = env::var("CARGO_FEATURE_VERSATILEAB").is_ok();
    let is_zynqmp = env::var("CARGO_FEATURE_ZYNQMP").is_ok();

    if has_versatileab == is_zynqmp {
        panic!(
            "Select exactly one board feature: `versatileab` or `zynqmp`. \
             For ZynqMP use `--no-default-features --features embedded,zynqmp`."
        );
    }

    // ZynqMP RPU Configuration Constraints (Lockstep Mode Only)
    //
    // This implementation targets Cortex-R5 in LOCKSTEP mode exclusively.
    // In lockstep, both R5 cores execute identical instructions in parallel
    // for redundancy - they appear as a single logical CPU to software.
    // Split mode (dual independent R5 cores) is NOT supported.
    //
    // Rationale:
    // - Global `_tx_gic_iar` variable assumes single-core execution
    // - Deferred EOI pattern has no per-core state tracking
    // - Critical section uses interrupt disable (not spinlocks)

    if is_zynqmp && env::var("CARGO_FEATURE_TX_ENABLE_IRQ_NESTING").is_ok() {
        panic!("ZynqMP: IRQ nesting requires per-nesting-level IAR storage - not implemented");
    }
    if is_zynqmp && env::var("CARGO_FEATURE_SMP").is_ok() {
        panic!("ZynqMP: SMP not supported - this implementation targets lockstep mode only");
    }

    // Select linker script based on board feature.
    // Pass it directly via rustc-link-arg to avoid search-order ambiguity
    // (the crate root contains linker.ld for VersatileAB, which the linker
    // would find before any OUT_DIR copy).
    let linker_script = if is_zynqmp {
        crate_dir.join("linker_zynqmp.ld")
    } else {
        crate_dir.join("linker.ld")
    };
    println!("cargo:rustc-link-arg=-T{}", linker_script.display());
    println!("cargo:rerun-if-changed=linker.ld");
    println!("cargo:rerun-if-changed=linker_zynqmp.ld");

    // Build ThreadX as static library
    let tx_common_dir = crate_dir.join("../threadx/common/src");
    let tx_common_inc = crate_dir.join("../threadx/common/inc");
    let tx_port_dir = crate_dir.join("../threadx/ports/cortex_r5/gnu/src");
    let tx_port_inc = crate_dir.join("../threadx/ports/cortex_r5/gnu/inc");

    // All port files used for both platforms (upstream ThreadX supports GIC deferred EOI)
    let port_files: Vec<_> = TX_PORT_FILES.iter().map(|&s| tx_port_dir.join(s)).collect();

    let mut threadx_build = cc::Build::new();
    threadx_build
        .include(&tx_common_inc)
        .include(&tx_port_inc)
        .flag("-g")
        .flag("-ffunction-sections")
        .flag("-fdata-sections")
        .flag("-fno-unwind-tables")
        .flag("-fno-asynchronous-unwind-tables")
        // Size optimizations (code-path only; defines that alter struct layout
        // like TX_NO_FILEX_POINTER or TX_DISABLE_NOTIFY_CALLBACKS must NOT be
        // used unless threadx-sys bindings are regenerated with matching defines)
        .define("TX_DISABLE_ERROR_CHECKING", "1")
        .define("TX_DISABLE_REDUNDANT_CLEARING", "1")
        .define("TX_ENABLE_VFP_SUPPORT", "1")
        .files(&port_files)
        .files(TX_COMMON_FILES.iter().map(|&s| tx_common_dir.join(s)));

    // Enable GIC deferred EOI for ZynqMP (upstream ThreadX feature)
    if is_zynqmp {
        threadx_build.define("TX_GIC_DEFERRED_EOI", "1");
        // ZynqMP GIC CPU interface is at 0xF902_0000 (not default 0xF9001000)
        threadx_build.define("TX_GIC_CPU_INTERFACE_BASE", "0xF9020000");
    }

    threadx_build.compile("threadx");

    // Build startup code
    let mut startup_build = cc::Build::new();
    startup_build
        .include(&tx_common_inc)
        .include(&tx_port_inc)
        .flag("-g")
        .flag("-ffunction-sections")
        .flag("-fdata-sections")
        .flag("-fno-unwind-tables")
        .flag("-fno-asynchronous-unwind-tables")
        .file("src/tx_initialize_low_level.S");

    // Define ZYNQMP for assembly conditional compilation
    if is_zynqmp {
        startup_build.define("ZYNQMP", "1");
    }

    startup_build.compile("startup");

    Ok(())
}
