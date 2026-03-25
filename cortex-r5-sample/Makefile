PROFILE = release
TARGET = ../target/armv7r-none-eabihf/$(PROFILE)/cortex-r5-sample

OPT_PROFILE = --$(PROFILE)

# Default board: versatileab
BOARD ?= versatileab

# Xilinx QEMU fork (built from vendor/xilinx-qemu with RPU GIC support)
XILINX_QEMU ?= ../vendor/xilinx-qemu/build/qemu-system-aarch64

# Board-specific cargo feature flags and QEMU commands
ifeq ($(BOARD),versatileab)
  BOARD_FEATURES = --no-default-features --features embedded,versatileab
  QEMU_RUN = qemu-system-arm -machine versatileab -cpu cortex-r5f \
    -semihosting -nographic -kernel $(TARGET)
else ifeq ($(BOARD),zynqmp)
  BOARD_FEATURES = --no-default-features --features embedded,zynqmp
  # xlnx-zcu102: Zynq UltraScale+ with dual R5F (RPU) + quad A53 (APU)
  # -smp 6 = 4 APU + 2 RPU CPUs; boot-cpu selects R5 as boot target
  QEMU_RUN = $(XILINX_QEMU) -M xlnx-zcu102 -smp 6 -m 4G -nographic \
    -semihosting \
    -global xlnx-zynqmp.boot-cpu=rpu-cpu[0] \
    -global cortex-r5f-arm-cpu.start-powered-off=false \
    -kernel $(TARGET)
endif

all: build

.PHONY: build
build:
	cargo build $(OPT_PROFILE) $(BOARD_FEATURES)

run: build
	$(QEMU_RUN)

versatileab:
	$(MAKE) BOARD=versatileab all

run-versatileab:
	$(MAKE) BOARD=versatileab run

zynqmp:
	$(MAKE) BOARD=zynqmp all

run-zynqmp:
	$(MAKE) BOARD=zynqmp run

run-zynqmp-gdb:
	@echo "Connect with: arm-none-eabi-gdb $(TARGET) -ex 'target remote :1234'"
	$(MAKE) BOARD=zynqmp _run-gdb

_run-gdb: build
	$(QEMU_RUN) -S -gdb tcp::1234

clean:
	cargo clean

.PHONY: all run clean versatileab run-versatileab zynqmp run-zynqmp run-zynqmp-gdb _run-gdb
