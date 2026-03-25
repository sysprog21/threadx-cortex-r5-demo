/*
 * Linker script for ThreadX + Rust on Zynq UltraScale+ RPU (Cortex-R5F).
 *
 * Memory layout for RPU (Real-time Processing Unit):
 *
 *   VECTORS - Exception vectors at 0x0 (low vectors, SCTLR.V=0)
 *   DDR     - Code, data, BSS, ThreadX heap
 *   STACK   - System stacks in DDR
 *
 * This layout uses LOW vectors (0x0) for Xilinx QEMU compatibility.
 * Xilinx QEMU's xlnx-zcu102 starts R5 with SCTLR.V=0 (low vectors).
 *
 * ZynqMP RPU memory map:
 *   Low vectors: 0x0000_0000 - 0x0000_003F (64 bytes)
 *   Startup code: 0x0000_0040 - 0x0000_00FF
 *   DDR: 0x0010_0000 onwards
 *
 * QEMU Notes:
 *
 * ZynqMP QEMU (xlnx-zcu102) is complex due to multi-core A53+R5 architecture.
 * The R5 cores are held in reset and must be released by the A53 writing to
 * the RST_LPD_TOP register. An APU shim is required to boot R5 on QEMU.
 *
 * For development/testing, use VersatileAB which provides simpler single-core
 * emulation:
 *
 *   cargo build --release  # VersatileAB (default)
 *   make run
 *
 * For ZynqMP hardware/simulation, this linker script provides the correct
 * memory layout for lockstep Cortex-R5 operation.
 */

MEMORY {
    /* Exception vectors at low vector address (SCTLR.V=0 on R5) */
    VECTORS : ORIGIN = 0x00000000, LENGTH = 64

    /* Startup code immediately after vectors */
    STARTUP : ORIGIN = 0x00000040, LENGTH = 192

    /* Main code, data in DDR */
    DDR : ORIGIN = 0x00100000, LENGTH = 14M

    /* Stack region in DDR */
    STACK : ORIGIN = 0x00F00000, LENGTH = 64K
}

ENTRY(_start)

SECTIONS {
    /* Exception vectors at low vector address - MUST be first */
    .vectors ORIGIN(VECTORS) : {
        KEEP(*(.vectors))
    } > VECTORS

    /* Startup code immediately after vectors */
    .startup ORIGIN(STARTUP) : {
        KEEP(*(.text.startup))
    } > STARTUP

    /* Main code section */
    .text ORIGIN(DDR) : {
        *(.text .text*)
    } > DDR

    /* Read-only data */
    .rodata : {
        *(.rodata .rodata*)
    } > DDR

    /* Initialized data */
    .data : {
        *(.data .data*)
    } > DDR

    /* Uninitialized data (BSS) - aligned for 4-byte store loop in startup */
    .bss : ALIGN(4) {
        _bss_start = .;
        *(.bss .bss* COMMON)
        . = ALIGN(4);
        _bss_end = .;
    } > DDR

    /DISCARD/ : {
        *(.note .note*)
        *(.comment)
        *(.ARM.attributes)
    }

    /* ThreadX heap starts here */
    . = ALIGN(16);
    _end = .;
    __end__ = .;

    /* System stacks */
    .stack (NOLOAD) : {
        _stack_bottom = ABSOLUTE(.);
        . = ALIGN(8);
        . += LENGTH(STACK);
        _sp = . - 16;
        _stack_top = ABSOLUTE(.);
    } > STACK
}
