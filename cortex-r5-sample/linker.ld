/*
 * Linker script for ThreadX + Rust on QEMU VersatileAB / ARM Cortex-R5.
 *
 * Memory is split into three regions modelling real Cortex-R5 TCM layout:
 *   ITCM - vectors and startup code (zero-wait-state instruction fetch)
 *   DTCM - system stacks (zero-wait-state data access)
 *   RAM  - code, data, BSS, ThreadX heap
 *
 * On QEMU VersatileAB all three regions map to flat RAM.  On real silicon
 * with tightly-coupled memories the layout gives deterministic interrupt
 * latency and faster context switches.
 */

MEMORY {
    ITCM : ORIGIN = 0x00000000, LENGTH = 64K
    DTCM : ORIGIN = 0x00020000, LENGTH = 64K
    RAM  : ORIGIN = 0x00100000, LENGTH = 15M
}

ENTRY(_start)

SECTIONS {
    /* Exception vectors and early startup in ITCM for deterministic fetch */
    .startup ORIGIN(ITCM) : {
        *(.text.startup)
    } > ITCM

    .text : { *(.text .text*) } > RAM
    .rodata : { *(.rodata .rodata*) } > RAM
    .data : { *(.data .data*) } > RAM
    .bss : { *(.bss .bss* COMMON) } > RAM
    /DISCARD/ : {
        *(.note .note*)
    }

    /* First free RAM address -- ThreadX heap starts here */
    . = ALIGN(16);
    _end = .; __end__ = .;

    /* System stacks in DTCM for deterministic data access.
     * ARM mode stacks (FIQ/IRQ/ABT/UND/SYS/SVC) are carved from this
     * region by tx_initialize_low_level.S, working downward from _sp.
     */
    .stack (NOLOAD) : {
        _stack_bottom = ABSOLUTE(.);
        . = ALIGN(8);
        . += LENGTH(DTCM);
        _sp = . - 16;
        _stack_top = ABSOLUTE(.);
    } > DTCM
}
