//! Hardware memory map and configuration constants for Zynq UltraScale+ RPU (Cortex-R5F).
//!
//! This module centralizes all hardware base addresses and register offsets
//! for the xlnx-zcu102 QEMU machine running on the Real-time Processing Unit (RPU).
//!
//! # Memory Map (Zynq UltraScale+ RPU)
//!
//! | Peripheral     | Base Address   | IRQ (SPI) |
//! |----------------|----------------|-----------|
//! | UART0          | 0xFF00_0000    | 21        |
//! | UART1          | 0xFF01_0000    | 22        |
//! | TTC0           | 0xFF11_0000    | 36-38     |
//! | TTC1           | 0xFF12_0000    | 39-41     |
//! | TTC2           | 0xFF13_0000    | 42-44     |
//! | TTC3           | 0xFF14_0000    | 45-47     |
//! | GIC Distributor| 0xF901_0000    | -         |
//! | GIC CPU Interface| 0xF902_0000  | -         |
//!
//! # RPU Memory Map
//!
//! | Region    | Address Range              | Size  |
//! |-----------|----------------------------|-------|
//! | ATCM      | 0xFFE0_0000 - 0xFFE0_FFFF  | 64KB  |
//! | BTCM      | 0xFFE2_0000 - 0xFFE2_FFFF  | 64KB  |
//! | OCM       | 0xFFFC_0000 - 0xFFFF_FFFF  | 256KB |
//! | DDR       | 0x0000_0000 - 0x7FFF_FFFF  | 2GB   |

// =============================================================================
// UART (Cadence/Xilinx UART)
// =============================================================================

/// Cadence UART0 base address (Zynq UltraScale+ RPU)
pub const UART0_BASE: usize = 0xFF00_0000;

/// Cadence UART1 base address
pub const UART1_BASE: usize = 0xFF01_0000;

/// UART0 interrupt number (GIC SPI 21)
/// Note: SPI interrupts start at 32, so actual GIC IRQ = 32 + 21 = 53
pub const UART0_IRQ: u32 = 53;

/// Cadence UART register definitions
pub mod uart {
    /// Control register offset
    pub const CONTROL_OFFSET: usize = 0x00;
    /// Mode register offset
    pub const MODE_OFFSET: usize = 0x04;
    /// Interrupt enable register offset
    pub const IER_OFFSET: usize = 0x08;
    /// Interrupt disable register offset
    pub const IDR_OFFSET: usize = 0x0C;
    /// Interrupt mask register offset
    pub const IMR_OFFSET: usize = 0x10;
    /// Channel interrupt status register offset
    pub const ISR_OFFSET: usize = 0x14;
    /// Baud rate generator register offset
    pub const BAUD_GEN_OFFSET: usize = 0x18;
    /// Receiver timeout register offset
    pub const RX_TIMEOUT_OFFSET: usize = 0x1C;
    /// Receiver FIFO trigger level register offset
    pub const RX_FIFO_TRIG_OFFSET: usize = 0x20;
    /// Modem control register offset
    pub const MODEM_CTRL_OFFSET: usize = 0x24;
    /// Modem status register offset
    pub const MODEM_STS_OFFSET: usize = 0x28;
    /// Channel status register offset
    pub const CHANNEL_STS_OFFSET: usize = 0x2C;
    /// TX/RX FIFO register offset
    pub const FIFO_OFFSET: usize = 0x30;
    /// Baud rate divider register offset
    pub const BAUD_DIV_OFFSET: usize = 0x34;
    /// Flow control delay register offset
    pub const FLOW_DELAY_OFFSET: usize = 0x38;
    /// TX FIFO trigger level register offset
    pub const TX_FIFO_TRIG_OFFSET: usize = 0x44;

    // Control register bits
    /// Stop transmitter
    pub const CTRL_STPBRK: u32 = 1 << 8;
    /// Start break
    pub const CTRL_STTBRK: u32 = 1 << 7;
    /// Restart receiver timeout counter
    pub const CTRL_RSTTO: u32 = 1 << 6;
    /// Transmit disable
    pub const CTRL_TXDIS: u32 = 1 << 5;
    /// Transmit enable
    pub const CTRL_TXEN: u32 = 1 << 4;
    /// Receive disable
    pub const CTRL_RXDIS: u32 = 1 << 3;
    /// Receive enable
    pub const CTRL_RXEN: u32 = 1 << 2;
    /// Software reset for TX
    pub const CTRL_TXRES: u32 = 1 << 1;
    /// Software reset for RX
    pub const CTRL_RXRES: u32 = 1 << 0;

    // Channel status register bits
    /// TX FIFO nearly full
    pub const STS_TNFUL: u32 = 1 << 14;
    /// TX FIFO trigger level reached
    pub const STS_TTRIG: u32 = 1 << 13;
    /// RX FIFO fill level trigger
    pub const STS_FLOWDEL: u32 = 1 << 12;
    /// TX state machine active
    pub const STS_TACTIVE: u32 = 1 << 11;
    /// RX state machine active
    pub const STS_RACTIVE: u32 = 1 << 10;
    /// TX FIFO full
    pub const STS_TXFULL: u32 = 1 << 4;
    /// TX FIFO empty
    pub const STS_TXEMPTY: u32 = 1 << 3;
    /// RX FIFO full
    pub const STS_RXFULL: u32 = 1 << 2;
    /// RX FIFO empty
    pub const STS_RXEMPTY: u32 = 1 << 1;
    /// RX FIFO trigger level reached
    pub const STS_RTRIG: u32 = 1 << 0;

    // Mode register bits
    /// Channel mode: normal
    pub const MODE_CHMODE_NORMAL: u32 = 0 << 8;
    /// Channel mode: automatic echo
    pub const MODE_CHMODE_ECHO: u32 = 1 << 8;
    /// Channel mode: local loopback
    pub const MODE_CHMODE_LOOPBACK: u32 = 2 << 8;
    /// Channel mode: remote loopback
    pub const MODE_CHMODE_REMOTE_LOOPBACK: u32 = 3 << 8;
    /// Stop bits: 1
    pub const MODE_NBSTOP_1: u32 = 0 << 6;
    /// Parity: none
    pub const MODE_PAR_NONE: u32 = 4 << 3;
    /// Character length: 8 bits
    pub const MODE_CHRL_8: u32 = 0 << 1;
    /// Clock source: uart_ref_clk
    pub const MODE_CLKS_REF: u32 = 0 << 0;
}

// =============================================================================
// Timer (Cadence Triple Timer Counter - TTC)
// =============================================================================

/// TTC0 base address (3 timer channels)
pub const TTC0_BASE: usize = 0xFF11_0000;

/// TTC1 base address
pub const TTC1_BASE: usize = 0xFF12_0000;

/// TTC2 base address
pub const TTC2_BASE: usize = 0xFF13_0000;

/// TTC3 base address
pub const TTC3_BASE: usize = 0xFF14_0000;

/// TTC0 Timer0 interrupt (GIC SPI 36)
pub const TTC0_TMR0_IRQ: u32 = 68; // 32 + 36

/// TTC register offsets (per channel, channels are 0xC apart)
pub mod ttc {
    /// Clock control register offset
    pub const CLK_CTRL_OFFSET: usize = 0x00;
    /// Counter control register offset
    pub const CNT_CTRL_OFFSET: usize = 0x0C;
    /// Counter value register offset
    pub const CNT_VALUE_OFFSET: usize = 0x18;
    /// Interval counter register offset
    pub const INTERVAL_OFFSET: usize = 0x24;
    /// Match 1 register offset
    pub const MATCH1_OFFSET: usize = 0x30;
    /// Match 2 register offset
    pub const MATCH2_OFFSET: usize = 0x3C;
    /// Match 3 register offset
    pub const MATCH3_OFFSET: usize = 0x48;
    /// Interrupt status register offset
    pub const ISR_OFFSET: usize = 0x54;
    /// Interrupt enable register offset
    pub const IER_OFFSET: usize = 0x60;

    // Counter control bits
    /// Disable counter
    pub const CNT_CTRL_DIS: u32 = 1 << 0;
    /// Interval mode (vs overflow)
    pub const CNT_CTRL_INT: u32 = 1 << 1;
    /// Decrement (vs increment)
    pub const CNT_CTRL_DEC: u32 = 1 << 2;
    /// Match mode enable
    pub const CNT_CTRL_MATCH: u32 = 1 << 3;
    /// Reset counter
    pub const CNT_CTRL_RST: u32 = 1 << 4;

    // Interrupt bits
    /// Interval interrupt
    pub const INT_INTERVAL: u32 = 1 << 0;
    /// Match 1 interrupt
    pub const INT_MATCH1: u32 = 1 << 1;
    /// Match 2 interrupt
    pub const INT_MATCH2: u32 = 1 << 2;
    /// Match 3 interrupt
    pub const INT_MATCH3: u32 = 1 << 3;
    /// Counter overflow interrupt
    pub const INT_OVERFLOW: u32 = 1 << 4;
    /// Event timer overflow interrupt
    pub const INT_EVENT: u32 = 1 << 5;

    // Clock control bits
    /// Prescaler enable
    pub const CLK_CTRL_PS_EN: u32 = 1 << 0;
    /// Prescaler value shift (bits 1-4)
    pub const CLK_CTRL_PS_SHIFT: u32 = 1;
    /// External clock select
    pub const CLK_CTRL_EXT: u32 = 1 << 5;
    /// External clock edge
    pub const CLK_CTRL_EDGE: u32 = 1 << 6;
}

// =============================================================================
// GIC (Generic Interrupt Controller v1 / PL390)
// =============================================================================

/// GIC Distributor base address
pub const GICD_BASE: usize = 0xF901_0000;

/// GIC CPU Interface base address
pub const GICC_BASE: usize = 0xF902_0000;

/// GIC CPU target mask for the local R5 core.
///
/// QEMU xlnx-zcu102 uses a unified GIC where CPUs 0-3 = A53 (APU) and
/// CPUs 4-5 = R5 (RPU), so R5-0 is CPU 4 → target bit 4 → 0x10.
/// On real ZynqMP hardware the RPU has its own GIC view where the local
/// core is CPU 0 → target bit 0 → 0x01.
///
/// Change this constant when moving from QEMU to silicon.
pub const GIC_CPU_TARGET: u8 = 0x10;

/// GIC register offsets
pub mod gic {
    // Distributor registers
    /// Distributor Control Register
    pub const GICD_CTLR: usize = 0x000;
    /// Interrupt Controller Type Register
    pub const GICD_TYPER: usize = 0x004;
    /// Distributor Implementer ID Register
    pub const GICD_IIDR: usize = 0x008;
    /// Interrupt Set-Enable Registers (base)
    pub const GICD_ISENABLER: usize = 0x100;
    /// Interrupt Clear-Enable Registers (base)
    pub const GICD_ICENABLER: usize = 0x180;
    /// Interrupt Set-Pending Registers (base)
    pub const GICD_ISPENDR: usize = 0x200;
    /// Interrupt Clear-Pending Registers (base)
    pub const GICD_ICPENDR: usize = 0x280;
    /// Interrupt Priority Registers (base)
    pub const GICD_IPRIORITYR: usize = 0x400;
    /// Interrupt Processor Targets Registers (base)
    pub const GICD_ITARGETSR: usize = 0x800;
    /// Interrupt Configuration Registers (base)
    pub const GICD_ICFGR: usize = 0xC00;
    /// Software Generated Interrupt Register
    pub const GICD_SGIR: usize = 0xF00;

    // CPU Interface registers
    /// CPU Interface Control Register
    pub const GICC_CTLR: usize = 0x000;
    /// Interrupt Priority Mask Register
    pub const GICC_PMR: usize = 0x004;
    /// Binary Point Register
    pub const GICC_BPR: usize = 0x008;
    /// Interrupt Acknowledge Register
    pub const GICC_IAR: usize = 0x00C;
    /// End of Interrupt Register
    pub const GICC_EOIR: usize = 0x010;
    /// Running Priority Register
    pub const GICC_RPR: usize = 0x014;
    /// Highest Priority Pending Interrupt Register
    pub const GICC_HPPIR: usize = 0x018;

    // Control bits
    /// Enable Group 0 interrupts (Distributor)
    pub const GICD_CTLR_ENABLE: u32 = 1 << 0;
    /// Enable signaling of interrupts (CPU Interface)
    pub const GICC_CTLR_ENABLE: u32 = 1 << 0;
}

// =============================================================================
// Memory Regions
// =============================================================================

/// ATCM (Tightly Coupled Memory A) base address
pub const ATCM_BASE: usize = 0xFFE0_0000;

/// ATCM size in bytes (64KB)
pub const ATCM_SIZE: usize = 64 * 1024;

/// BTCM (Tightly Coupled Memory B) base address
pub const BTCM_BASE: usize = 0xFFE2_0000;

/// BTCM size in bytes (64KB)
pub const BTCM_SIZE: usize = 64 * 1024;

/// OCM (On-Chip Memory) base address
pub const OCM_BASE: usize = 0xFFFC_0000;

/// OCM size in bytes (256KB)
pub const OCM_SIZE: usize = 256 * 1024;

/// DDR base address
pub const DDR_BASE: usize = 0x0000_0000;

/// DDR size in bytes (2GB for QEMU xlnx-zcu102)
pub const DDR_SIZE: usize = 2 * 1024 * 1024 * 1024;
