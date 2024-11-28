/*
 * Linker script for executing ThreadX and Rust applications on QEMU's VersatileAB
 * powered by Arm Cortex-R5 processor.
*/

MEMORY {
    RAM : ORIGIN = 0, LENGTH = 0x1000000
}

ENTRY(_start)
SECTIONS {
    .startup ORIGIN(RAM) : {
        *(.text.startup)
    } > RAM
    .text : { *(.text .text*) } > RAM
    .rodata : { *(.rodata .rodata*) } > RAM
    .data : { *(.data .data*) } > RAM
    .bss : { *(.bss .bss* COMMON) } > RAM
    /DISCARD/ : {
        *(.note .note*)
    }

    . = ALIGN(16);
    .stack : {
        _stack_bottom = ABSOLUTE(.) ;
        /* Allocate space for the stack, ensuring it is sufficiently large to
         * accommodate the IRQ, FIQ, and SYS stacks when nested interrupts are
         * enabled.
         */
        .   =  ALIGN(8) ;
        .   += 0x100000;
        _sp =  . - 16 ;
        _stack_top = ABSOLUTE(.) ;
    } > RAM

  _end = .; __end__ = . ;
}
