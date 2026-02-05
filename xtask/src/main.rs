//! xtask - Build and test automation for ThreadX Cortex-R5 demo
//!
//! Usage:
//!   cargo xtask run      # Build and run on QEMU
//!   cargo xtask smoke    # Run QEMU smoke test (marker validation)
//!   cargo xtask test     # Run host tests (compile-fail, doctests, unit tests)
//!   cargo xtask size     # Print binary size breakdown
//!   cargo xtask help     # Show this help
//!
//! The smoke test validates the boot path by checking for deterministic
//! markers in UART output. This catches linker script regressions, startup
//! faults, and thread initialization failures.

use anyhow::{bail, Context, Result};
use std::io::{BufRead, BufReader};
use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::sync::mpsc;
use std::time::{Duration, Instant};

const ELF_PATH: &str = "target/armv7r-none-eabihf/release/cortex-r5-sample";

/// Boot phase markers emitted by the application.
/// These form a ladder of validation points from boot through runtime.
///
/// Phase 1: Initialization (before scheduler)
/// Phase 2: Primitives creation (Scenario A)
/// Phase 3: Thread spawn + scheduler start
/// Phase 4: Timer validation
/// Phase 5: Producer-consumer pipeline (Scenario B)
/// Phase 6: Mutex contention + priority inheritance (Scenario C)
/// Phase 7: Lock-free SPSC ring + event flags (Scenario D)
const MARKERS: &[&str] = &[
    // Phase 1: Initialization
    "Running ThreadX",       // kmain() entry
    "tx_application_define", // Thread creation phase
    // Phase 2: Primitives (Scenario A - creation only)
    "PRIMITIVES_OK", // All primitives created
    // Phase 3: Thread spawn
    "Thread spawned", // Thread creation works
    "APP_READY",      // Initialization complete
    // Phase 4: Runtime validation
    "SCHED_RUNNING", // Scheduler dispatched thread
    "TICK_OK",       // Timer interrupt works
    // Phase 5: Producer-consumer pipeline (Scenario B)
    "PIPE_OK", // Pipeline FIFO ordering verified
    // Phase 6: Mutex contention (Scenario C)
    "MTX_HELD", // Producer holds mutex under contention
    "MTX_OK",   // Mutual exclusion + priority inheritance verified
    // Phase 7: Lock-free SPSC ring + event flags (Scenario D)
    "SPSC_OK", // SPSC data integrity verified
];

/// Failure patterns that indicate critical faults.
/// Any of these in UART output = immediate test failure.
const FAILURE_PATTERNS: &[&str] = &[
    "PANIC",
    "panic",
    "!!! STACK OVERFLOW",
    "=== DATA ABORT ===",
    "=== PREFETCH ABORT ===",
    "=== UNDEFINED INSTRUCTION ===",
    "=== SVC EXCEPTION ===",
    "Failed to", // ThreadX API failures
];

/// Number of recent UART lines to keep for diagnostics on failure.
const TAIL_LINES: usize = 20;

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let cmd = args.get(1).map(|s| s.as_str()).unwrap_or("help");

    match cmd {
        "run" => qemu_run()?,
        "smoke" => qemu_smoke()?,
        "test" => host_test()?,
        "size" => print_size()?,
        "help" | "--help" | "-h" => print_help(),
        _ => {
            eprintln!("Unknown command: {}", cmd);
            print_help();
            std::process::exit(1);
        }
    }

    Ok(())
}

fn print_help() {
    eprintln!(
        r#"xtask - ThreadX Cortex-R5 build automation

USAGE:
    cargo xtask <command>

COMMANDS:
    run      Build release binary and run on QEMU (interactive)
    smoke    Run QEMU smoke test with runtime validation
    test     Run host tests (compile-fail, doctests, unit tests)
    size     Print binary size breakdown
    help     Show this help

ENVIRONMENT:
    QEMU_TIMEOUT    Timeout in seconds (default: 30)
    QEMU_VERBOSE    Show full UART output

SMOKE TEST MARKERS:
    Running ThreadX → PRIMITIVES_OK → APP_READY → TICK_OK → PIPE_OK → MTX_OK → SPSC_OK

EXAMPLES:
    cargo xtask smoke                  # Validate boot + runtime
    cargo xtask test                   # Run host tests
    QEMU_VERBOSE=1 cargo xtask smoke   # Debug output
"#
    );
}

/// Detect the host target triple via `rustc -vV`.
fn host_triple() -> Result<String> {
    let output = Command::new("rustc")
        .args(["-vV"])
        .output()
        .context("Failed to run rustc -vV")?;
    let stdout = String::from_utf8_lossy(&output.stdout);
    for line in stdout.lines() {
        if let Some(host) = line.strip_prefix("host: ") {
            return Ok(host.to_string());
        }
    }
    bail!("Could not determine host triple from rustc -vV");
}

/// Run host tests (compile-fail, doctests, unit tests).
///
/// The cortex-r5-sample/.cargo/config.toml forces the ARM target for embedded
/// builds. Host tests need the native target to access std (required by dev
/// dependencies like trybuild). This command handles the override automatically.
fn host_test() -> Result<()> {
    let host = host_triple()?;
    eprintln!("[xtask] Running host tests (target: {})...", host);

    let status = Command::new("cargo")
        .args([
            "test",
            "-p",
            "cortex-r5-sample",
            "--no-default-features",
            "--features",
            "host-test",
            "--target",
            &host,
        ])
        .status()
        .context("Failed to run cargo test")?;

    if !status.success() {
        bail!("Host tests failed");
    }

    eprintln!("[xtask] Host tests PASSED");
    Ok(())
}

/// Build release binary using cargo
///
/// Must be run from cortex-r5-sample directory to pick up .cargo/config.toml
/// which specifies the ARM target and rustflags.
fn cargo_build() -> Result<PathBuf> {
    eprintln!("[xtask] Building release binary...");

    // Run cargo from cortex-r5-sample directory to use its .cargo/config.toml
    // which specifies target=armv7r-none-eabihf and linker script
    let status = Command::new("cargo")
        .args(["build", "--release"])
        .current_dir("cortex-r5-sample")
        .status()
        .context("Failed to run cargo build")?;

    if !status.success() {
        bail!("cargo build failed with status: {}", status);
    }

    let elf = PathBuf::from(ELF_PATH);
    if !elf.exists() {
        bail!("ELF binary not found at {}", ELF_PATH);
    }

    eprintln!("[xtask] Built: {}", ELF_PATH);
    Ok(elf)
}

/// Run QEMU interactively (for development)
fn qemu_run() -> Result<()> {
    let elf = cargo_build()?;

    eprintln!("[xtask] Running QEMU (Ctrl+A X to exit)...");

    let status = Command::new("qemu-system-arm")
        .args([
            "-machine",
            "versatileab",
            "-cpu",
            "cortex-r5f",
            "-semihosting",
            "-nographic",
            "-kernel",
            elf.to_str().unwrap(),
        ])
        .status()
        .context("Failed to run QEMU")?;

    if !status.success() {
        bail!("QEMU exited with status: {}", status);
    }

    Ok(())
}

/// QEMU smoke test with marker-driven validation.
///
/// Monitors UART output for boot phase markers. Exits successfully when
/// the success marker is seen, or fails on timeout/panic.
///
/// Uses a reader thread to avoid blocking on BufRead::lines() when QEMU
/// stops producing output. Stderr is sent to null to prevent pipe stalls.
fn qemu_smoke() -> Result<()> {
    let elf = cargo_build()?;

    let timeout_secs: u64 = std::env::var("QEMU_TIMEOUT")
        .unwrap_or_else(|_| "30".to_string())
        .parse()
        .unwrap_or(30);
    let verbose = std::env::var("QEMU_VERBOSE").is_ok();
    let quick_mode = std::env::var("SMOKE_QUICK").is_ok();

    // In quick mode, stop at APP_READY (init validation only)
    // In full mode, stop at SPSC_OK (full runtime + SPSC ring validation)
    let success_marker = if quick_mode { "APP_READY" } else { "SPSC_OK" };

    eprintln!(
        "[xtask] Running smoke test (timeout: {}s, mode: {})...",
        timeout_secs,
        if quick_mode {
            "quick (init only)"
        } else {
            "full (runtime)"
        }
    );

    // Spawn QEMU with stdout piped for monitoring.
    // Stderr goes to null to prevent pipe buffer stalls.
    let mut child = Command::new("qemu-system-arm")
        .args([
            "-machine",
            "versatileab",
            "-cpu",
            "cortex-r5f",
            "-semihosting",
            "-nographic",
            "-kernel",
            elf.to_str().unwrap(),
        ])
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .context("Failed to spawn QEMU")?;

    let stdout = child.stdout.take().unwrap();
    let timeout = Duration::from_secs(timeout_secs);
    let start = Instant::now();

    // Read stdout lines on a separate thread to avoid blocking on lines()
    // when QEMU stops producing output (e.g., deadlock with no UART writes).
    let (tx, rx) = mpsc::channel::<String>();
    std::thread::spawn(move || {
        let reader = BufReader::new(stdout);
        for line in reader.lines() {
            match line {
                Ok(l) => {
                    if tx.send(l).is_err() {
                        break; // Receiver dropped
                    }
                }
                Err(_) => break,
            }
        }
    });

    // Track which markers we've seen
    let mut seen_markers: Vec<bool> = vec![false; MARKERS.len()];
    let mut saw_panic = false;
    let mut saw_success = false;
    // Ring buffer of recent lines for diagnostics on failure
    let mut recent_lines: Vec<String> = Vec::with_capacity(TAIL_LINES);

    // Monitor UART output with non-blocking receives
    loop {
        // Check timeout
        let remaining = timeout.checked_sub(start.elapsed());
        if remaining.is_none() {
            eprintln!("[xtask] TIMEOUT after {}s", timeout_secs);
            let _ = child.kill();
            break;
        }

        // Wait for next line with a bounded timeout (1s slices)
        let wait = remaining.unwrap().min(Duration::from_secs(1));
        let line = match rx.recv_timeout(wait) {
            Ok(l) => l,
            Err(mpsc::RecvTimeoutError::Timeout) => continue,
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        };

        // Always show output in verbose mode
        if verbose {
            eprintln!("  {}", line);
        }

        // Keep recent lines for failure diagnostics
        if recent_lines.len() >= TAIL_LINES {
            recent_lines.remove(0);
        }
        recent_lines.push(line.clone());

        // Check for critical faults (immediate failure)
        let mut fault_detected = false;
        for pattern in FAILURE_PATTERNS {
            if line.contains(pattern) {
                eprintln!("[xtask] FAULT detected ({}): {}", pattern, line);
                saw_panic = true;
                fault_detected = true;
                let _ = child.kill();
                break;
            }
        }
        if fault_detected {
            break;
        }

        // Check markers
        for (i, marker) in MARKERS.iter().enumerate() {
            if !seen_markers[i] && line.contains(marker) {
                seen_markers[i] = true;
                if !verbose {
                    eprintln!("[xtask] Marker: {}", marker);
                }
            }
        }

        // Success marker reached - stop QEMU
        if line.contains(success_marker) {
            saw_success = true;
            eprintln!("[xtask] {} detected - stopping QEMU", success_marker);
            let _ = child.kill();
            break;
        }
    }

    // Wait for QEMU to exit
    let _ = child.wait();

    // Report results
    eprintln!("\n[xtask] === Smoke Test Results ===");
    for (i, marker) in MARKERS.iter().enumerate() {
        let status = if seen_markers[i] { "✓" } else { "✗" };
        eprintln!("  {} {}", status, marker);
    }

    // Determine success/failure
    if saw_panic {
        bail!("Smoke test FAILED: fault detected");
    }

    if !saw_success {
        let missing: Vec<&str> = MARKERS
            .iter()
            .enumerate()
            .filter(|(i, _)| !seen_markers[*i])
            .map(|(_, m)| *m)
            .collect();

        if !missing.is_empty() {
            eprintln!("\nMissing markers: {:?}", missing);
        }

        // Print last N lines of UART output for diagnostics
        if !recent_lines.is_empty() {
            eprintln!("\nLast {} UART lines:", recent_lines.len());
            for l in &recent_lines {
                eprintln!("  {}", l);
            }
        }

        bail!(
            "Smoke test FAILED: {} not seen within {}s",
            success_marker,
            timeout_secs
        );
    }

    eprintln!("\n[xtask] Smoke test PASSED");
    Ok(())
}

/// Print binary size breakdown using arm-none-eabi-size
fn print_size() -> Result<()> {
    let elf = PathBuf::from(ELF_PATH);

    if !elf.exists() {
        eprintln!("[xtask] Binary not found, building first...");
        cargo_build()?;
    }

    eprintln!("[xtask] Binary size breakdown:");
    eprintln!();

    // Try arm-none-eabi-size first, fall back to size
    let size_cmd = if Command::new("arm-none-eabi-size")
        .arg("--version")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .is_ok()
    {
        "arm-none-eabi-size"
    } else {
        "size"
    };

    let output = Command::new(size_cmd)
        .args(["-A", "-d", elf.to_str().unwrap()])
        .output()
        .context("Failed to run size command")?;

    if !output.status.success() {
        bail!("size command failed");
    }

    // Print size output
    print!("{}", String::from_utf8_lossy(&output.stdout));

    // Also print total file size
    let metadata = std::fs::metadata(&elf).context("Failed to stat ELF")?;
    eprintln!("\nTotal file size: {} bytes", metadata.len());

    Ok(())
}
