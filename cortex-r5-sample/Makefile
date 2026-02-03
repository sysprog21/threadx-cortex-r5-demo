PROFILE = release
TARGET = ../target/armv7r-none-eabihf/$(PROFILE)/cortex-r5-sample

OPT_PROFILE = --$(PROFILE)

all: $(TARGET)

$(TARGET):
	cargo build $(OPT_PROFILE)

run: $(TARGET)
	cargo run $(OPT_PROFILE)

clean:
	cargo clean
