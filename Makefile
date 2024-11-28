threadx/common/inc/tx_api.h:
	git clone https://github.com/sysprog21/threadx

.DEFAULT_GOAL := all

all: threadx/common/inc/tx_api.h
	$(MAKE) -C cortex-r5-sample

run:
	$(MAKE) -C cortex-r5-sample run

clean:
	$(MAKE) -C cortex-r5-sample clean

distclean: clean
	rm -f cortex-r5-sample/Cargo.lock
	-rm -rf threadx
