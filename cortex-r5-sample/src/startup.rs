//! ARM Cortex-R5 startup code and exception vector table.
//!
//! This module contains the low-level assembly initialization sequence:
//! - Exception vector table configuration
//! - Stack pointer initialization
//! - VFP (Vector Floating Point) coprocessor enablement
//! - Jump to Rust kmain() entry point

core::arch::global_asm!(
    r#"

// Exception vector table - placed at 0xFFFF0000 (ZynqMP high vectors) or 0x0 (VersatileAB)
.section .vectors, "ax", %progbits
.global _vectors
.code 32
.align 0

_vectors:
    LDR     pc, STARTUP                     @ Reset goes to startup function 0x00
    LDR     pc, UNDEFINED                   @ Undefined handler              0x04
    LDR     pc, SWI                         @ Software interrupt handler     0x08
    LDR     pc, PREFETCH                    @ Prefetch exception handler     0x0C
    LDR     pc, ABORT                       @ Abort exception handler        0x10
    LDR     pc, RESERVED                    @ Reserved exception handler     0x14
    LDR     pc, IRQ                         @ IRQ interrupt handler          0x18
    LDR     pc, FIQ                         @ FIQ interrupt handler          0x1C

STARTUP:
    .word  _start                           @ Reset goes to C startup function
UNDEFINED:
    .word  __tx_undefined                   @ Undefined handler
SWI:
    .word  __tx_swi_interrupt               @ Software interrupt handler
PREFETCH:
    .word  __tx_prefetch_handler            @ Prefetch exception handler
ABORT:
    .word  __tx_abort_handler               @ Abort exception handler
RESERVED:
    .word  __tx_reserved_handler            @ Reserved exception handler
IRQ:
    .word  __tx_irq_handler                 @ IRQ interrupt handler
FIQ:
    .word  __tx_fiq_handler                 @ FIQ interrupt handler

// Startup code - placed after vectors
.section .text.startup
.global _start
.code 32
// Work around https://github.com/rust-lang/rust/issues/127269
.fpu vfp3-d16

_start:
    // Set stack pointer
    ldr sp, =_stack_top

    // Configure exception vector location (SCTLR.V bit)
    // ZynqMP R5 boots with high vectors (SCTLR.V=1, vectors at 0xFFFF0000)
    // We use low vectors (0x0) so clear V bit before any interrupts can occur
    mrc p15, 0, r0, c1, c0, 0   // Read SCTLR
    bic r0, r0, #(1 << 13)      // Clear V bit (bit 13) for low vectors
    mcr p15, 0, r0, c1, c0, 0   // Write SCTLR
    isb                          // Barrier to ensure SCTLR change takes effect

    // Allow VFP coprocessor access
    mrc p15, 0, r0, c1, c0, 2
    orr r0, r0, #0xF00000
    mcr p15, 0, r0, c1, c0, 2

    // Enable VFP
    mov r0, #0x40000000
    vmsr fpexc, r0

    // Zero BSS section (required for Rust statics on real hardware;
    // QEMU zeroes memory but warm resets on silicon leave stale data)
    ldr r0, =_bss_start
    ldr r1, =_bss_end
    mov r2, #0
1:  cmp r0, r1
    strlt r2, [r0], #4
    blt 1b

    // Jump to application
    bl kmain

    // In case the application returns, loop forever
    b .

"#
);
