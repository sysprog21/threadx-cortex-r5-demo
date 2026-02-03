threadx/common/inc/tx_api.h:
	git clone https://github.com/sysprog21/threadx

.DEFAULT_GOAL := all

all: threadx/common/inc/tx_api.h
	$(MAKE) -C cortex-r5-sample

run:
	$(MAKE) -C cortex-r5-sample run

# QEMU smoke test - validates boot + runtime (init → scheduler → timer IRQ)
smoke: all
	cargo run --manifest-path xtask/Cargo.toml -- smoke

# Binary size analysis
size: all
	cargo run --manifest-path xtask/Cargo.toml -- size

clean:
	$(MAKE) -C cortex-r5-sample clean
	cargo clean --manifest-path xtask/Cargo.toml 2>/dev/null || true

indent:
	(cd threadx-sys ; cargo fmt)
	(cd cortex-r5-sample ; cargo fmt)
	(cd xtask ; cargo fmt)

distclean: clean
	rm -f cortex-r5-sample/Cargo.lock
	rm -f xtask/Cargo.lock
	-rm -rf threadx

.PHONY: all run smoke size clean indent distclean
