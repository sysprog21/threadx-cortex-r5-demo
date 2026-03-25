# Top-level Makefile for ThreadX Cortex-R5 demo
#
# Usage:
#   make                    Build for VersatileAB (default)
#   make BOARD=zynqmp       Build for ZynqMP RPU
#   make run                Build and run on QEMU
#   make run BOARD=zynqmp   Build and run ZynqMP on Xilinx QEMU
#   make run-gdb            Build and run with GDB server (ZynqMP only)

BOARD ?= versatileab
SAMPLE_DIR = cortex-r5-sample

# ThreadX source (cloned on first build)
threadx/common/inc/tx_api.h:
	git clone https://github.com/sysprog21/threadx

.DEFAULT_GOAL := all

# --- Build and run ---

all: threadx/common/inc/tx_api.h
	$(MAKE) -C $(SAMPLE_DIR) BOARD=$(BOARD)

run: threadx/common/inc/tx_api.h
	$(MAKE) -C $(SAMPLE_DIR) BOARD=$(BOARD) run

run-gdb: threadx/common/inc/tx_api.h
	$(MAKE) -C $(SAMPLE_DIR) run-zynqmp-gdb

# --- Vendor QEMU (ZynqMP only) ---

build-qemu:
	./vendor/build-xilinx-qemu.sh

# --- Tooling ---

smoke: all
	cargo run --manifest-path xtask/Cargo.toml -- smoke

size: all
	cargo run --manifest-path xtask/Cargo.toml -- size

BINDGEN_OUT = target/armv7r-none-eabihf/release/build
bindgen: threadx/common/inc/tx_api.h
	cd $(SAMPLE_DIR) && cargo build --release --features threadx-sys/bindgen
	cp $$(ls -t $$(find $(BINDGEN_OUT) -path '*/threadx-sys-*/out/bindings.rs') | head -1) \
		threadx-sys/src/bindings.rs

# --- Maintenance ---

clean:
	$(MAKE) -C $(SAMPLE_DIR) clean
	cargo clean --manifest-path xtask/Cargo.toml 2>/dev/null || true

indent:
	(cd threadx-sys ; cargo fmt)
	(cd $(SAMPLE_DIR) ; cargo fmt)
	(cd xtask ; cargo fmt)

distclean: clean
	rm -f $(SAMPLE_DIR)/Cargo.lock
	rm -f xtask/Cargo.lock
	-rm -rf threadx

.PHONY: all run run-gdb build-qemu smoke size bindgen clean indent distclean
