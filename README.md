# Rust-written Real-Time Tasks on Eclipse ThreadX

This repository demonstrates how to compile and execute Rust-written tasks on
the [Eclipse ThreadX](https://github.com/eclipse-threadx/threadx) Real-Time
Operating System (RTOS). The primary target platform for this project is the Arm
Cortex-R5 processor, showcasing the integration of Rust with ThreadX for
high-performance real-time applications.

## Build and Run

Visit the [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
page and choose the AArch32 bare-metal target (`arm-none-eabi`) toolchain
variant compatible with your development environment.

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

## Licence

This project is available under a permissive MIT-style license.
Use of this source code is governed by a MIT license that can be found in the
[LICENSE](LICENSE) file.
