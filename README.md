# Rust-written Real-Time Tasks on Eclipse ThreadX

This repository demonstrates how to compile and execute Rust-written tasks on the
[Eclipse ThreadX](https://github.com/eclipse-threadx/threadx) Real-Time Operating System (RTOS).
The primary target platform for this project is the Arm Cortex-R5 processor,
showcasing the integration of Rust with ThreadX for high-performance real-time applications.

## Build and Run

Visit the [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
page and choose the AArch32 bare-metal target (`arm-none-eabi`) toolchain variant compatible with your development environment.

Install Rust via `rustup` (Recommended)
```shell
$ curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Then, enable bare-metal target for CPUs in the Armv7-R architecture family.
```shell
$ rustup target add armv7r-none-eabihf
```

Run `make` to build both ThreadX and Rust code from source.
Run `make run` to launch Rust-written tasks on ThreadX via QEMU.

Building does not require `libclang`; the `threadx-sys` crate ships pre-generated FFI bindings.
After upgrading ThreadX or modifying its headers, regenerate the bindings with:
```shell
$ make bindgen
```
This requires `libclang` to be installed.
The updated bindings are written to `threadx-sys/src/bindings.rs` and should be committed.

## Development

The project uses [cargo xtask](https://github.com/matklad/cargo-xtask) for build automation:

```shell
$ cargo xtask run      # Build release binary and run on QEMU (interactive)
$ cargo xtask smoke    # Marker-driven boot and runtime validation
$ cargo xtask size     # Binary size breakdown (text/data/bss per object)
$ cargo xtask help     # Show all commands and options
```

Makefile shortcuts are available for common operations:
```shell
$ make smoke           # Equivalent to cargo xtask smoke
$ make size            # Equivalent to cargo xtask size
```

Environment variables:
- `QEMU_TIMEOUT` - QEMU timeout in seconds (default: 30)
- `QEMU_VERBOSE` - Show full UART output during smoke tests
- `SMOKE_QUICK` - Skip slow markers for faster iteration

The smoke test validates deterministic markers emitted during boot and runtime:
`Running ThreadX` → `PRIMITIVES_OK` → `APP_READY` → `TICK_OK` → `PIPE_OK` → `MTX_OK`

## Licence

This project is available under a permissive MIT-style license.
Use of this source code is governed by a MIT license that can be found in the
[LICENSE](LICENSE) file.
