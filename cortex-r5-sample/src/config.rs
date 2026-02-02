//! Hardware memory map and configuration constants for QEMU VersatileAB.
//!
//! This module centralizes all hardware base addresses and register offsets,
//! eliminating magic numbers and constant duplication across driver modules.
//!
//! # Memory Map (QEMU VersatileAB)
//!
//! | Peripheral | Base Address | IRQ |
//! |------------|--------------|-----|
//! | UART0      | 0x101f_1000  | 12  |
//! | Timer0     | 0x101e_2000  | 4   |
//! | VIC        | 0x1014_0000  | -   |

/// PL011 UART0 base address (QEMU VersatileAB)
pub const UART0_BASE: usize = 0x101f_1000;

/// SP804 Timer0 base address (QEMU VersatileAB)
pub const TIMER0_BASE: usize = 0x101e_2000;

/// PL190 VIC (Vector Interrupt Controller) base address (QEMU VersatileAB)
pub const VIC_BASE: usize = 0x1014_0000;

/// UART0 interrupt number on VIC
pub const UART0_IRQ: u8 = 12;

/// Timer0 interrupt number on VIC
pub const TIMER0_IRQ: u8 = 4;

/// PL011 UART register offsets (word offsets, divide byte offset by 4)
pub mod uart {
    /// Data register offset (0x000 >> 2)
    pub const DATA_OFFSET: usize = 0x000 >> 2;

    /// Flag register offset (0x018 >> 2)
    pub const FLAG_OFFSET: usize = 0x018 >> 2;

    /// Control register offset (0x030 >> 2)
    pub const CONTROL_OFFSET: usize = 0x030 >> 2;

    /// Interrupt Mask Set/Clear register offset (0x038 >> 2)
    pub const IMSC_OFFSET: usize = 0x038 >> 2;

    /// Interrupt Clear register offset (0x044 >> 2)
    pub const ICR_OFFSET: usize = 0x044 >> 2;

    // Flag register bits

    /// TX FIFO Full flag (bit 5)
    pub const FLAG_TXFF: u32 = 1 << 5;

    // Control register bits

    /// UART Enable bit (bit 0)
    pub const CONTROL_UARTEN: u32 = 1 << 0;

    /// TX Enable bit (bit 8)
    pub const CONTROL_TXE: u32 = 1 << 8;

    // Interrupt mask bits

    /// TX Interrupt Mask (bit 5)
    pub const IMSC_TXIM: u32 = 1 << 5;

    /// Clear TX Interrupt (bit 5)
    pub const ICR_TXIC: u32 = 1 << 5;
}
