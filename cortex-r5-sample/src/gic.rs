//! Implementation for the ARM Generic Interrupt Controller (GIC).
//!
//! The Zynq UltraScale+ RPU uses a GICv1/v2 (PL390) with:
//! - Distributor at 0xF901_0000
//! - CPU Interface at 0xF902_0000
//!
//! GIC interrupt numbering:
//! - SGI (Software Generated): 0-15
//! - PPI (Private Peripheral): 16-31
//! - SPI (Shared Peripheral): 32-1019
//!
//! # Lockstep Mode Only
//!
//! This implementation targets Cortex-R5 in **lockstep mode** exclusively.
//! In lockstep, both R5 cores execute identical instructions in parallel
//! for hardware redundancy - they appear as a single logical CPU to software.
//!
//! Split mode (dual independent R5 cores) is NOT supported because:
//! - Global `_tx_gic_iar` has no per-core tracking
//! - Deferred EOI assumes single execution context
//! - GIC CPU interface targeting assumes single R5

#[cfg(feature = "zynqmp")]
use crate::config_zynqmp::{gic, GICC_BASE, GICD_BASE};

/// GIC Distributor driver.
pub struct GicDistributor<const ADDR: usize>();

/// GIC CPU Interface driver.
pub struct GicCpuInterface<const ADDR: usize>();

/// Combined GIC driver.
pub struct Gic {
    #[cfg(feature = "zynqmp")]
    _distributor: GicDistributor<GICD_BASE>,
    #[cfg(feature = "zynqmp")]
    _cpu_interface: GicCpuInterface<GICC_BASE>,
}

// External symbol for IAR value stored by assembly IRQ handler.
// Declared as mutable since assembly writes to it at IRQ entry.
#[cfg(feature = "zynqmp")]
extern "C" {
    /// IAR value stored by __tx_irq_handler in tx_initialize_low_level.S.
    /// Mutable because assembly writes without Rust synchronization.
    static mut _tx_gic_iar: u32;
}

#[cfg(feature = "zynqmp")]
impl Gic {
    /// Get IRQ ID from assembly-stored IAR.
    ///
    /// Returns the interrupt ID (bits 0-9 of IAR). The full IAR value
    /// (including CPUID bits 12:10 for SGIs) is preserved in `_tx_gic_iar`
    /// for deferred EOI. This function strips the CPUID field, so SGI
    /// source CPU information is not available through this API.
    ///
    /// # Safety
    ///
    /// Caller must ensure:
    /// - Called only from IRQ handler context (after assembly has written IAR)
    /// - Lockstep mode execution (R5 cores run in lockstep, not split mode)
    /// - IRQ nesting is disabled (TX_ENABLE_IRQ_NESTING not defined)
    ///
    /// This function relies on the global `_tx_gic_iar` variable which has no
    /// per-core tracking. It is safe only in lockstep mode where both R5 cores
    /// execute identical instructions and appear as a single logical CPU.
    #[inline]
    pub unsafe fn current_irq_id() -> u32 {
        // Use addr_of! to avoid creating a reference to mutable static
        core::ptr::read_volatile(core::ptr::addr_of!(_tx_gic_iar)) & 0x3FF
    }
    /// Create and initialize the GIC.
    ///
    /// # Safety
    /// Only construct one GIC object at any given time.
    pub unsafe fn new() -> Self {
        let gic = Gic {
            _distributor: GicDistributor(),
            _cpu_interface: GicCpuInterface(),
        };
        gic.init();
        gic
    }

    /// Initialize the GIC.
    fn init(&self) {
        // Initialize distributor
        GicDistributor::<GICD_BASE>::init();
        // Initialize CPU interface
        GicCpuInterface::<GICC_BASE>::init();
    }

    /// Maximum valid IRQ ID for GICv2 (1019, IDs 1020-1023 are special).
    const MAX_IRQ_ID: u32 = 1019;

    /// Validate IRQ ID is within GICv2 range (0-1019).
    /// Panics unconditionally (including release builds) on out-of-range IDs
    /// to prevent MMIO access to unmapped register space.
    #[inline]
    fn validate_irq(irq: u32) {
        assert!(
            irq <= Self::MAX_IRQ_ID,
            "IRQ ID out of GICv2 range (0-1019)"
        );
    }

    /// Enable a specific interrupt.
    ///
    /// # Panics
    /// Panics if `irq > 1019` (reserved/special IDs).
    pub fn enable_interrupt(&mut self, irq: u32) {
        Self::validate_irq(irq);
        GicDistributor::<GICD_BASE>::enable_interrupt(irq);
    }

    /// Disable a specific interrupt.
    ///
    /// # Panics
    /// Panics if `irq > 1019` (reserved/special IDs).
    pub fn disable_interrupt(&mut self, irq: u32) {
        Self::validate_irq(irq);
        GicDistributor::<GICD_BASE>::disable_interrupt(irq);
    }

    /// Clear a pending interrupt.
    ///
    /// # Panics
    /// Panics if `irq > 1019` (reserved/special IDs).
    pub fn clear_pending(&mut self, irq: u32) {
        Self::validate_irq(irq);
        GicDistributor::<GICD_BASE>::clear_pending(irq);
    }

    /// Set the priority of an interrupt.
    ///
    /// Lower values = higher priority. Priority is typically 0-255.
    ///
    /// # Panics
    /// Panics if `irq > 1019` (reserved/special IDs).
    pub fn set_priority(&mut self, irq: u32, priority: u8) {
        Self::validate_irq(irq);
        GicDistributor::<GICD_BASE>::set_priority(irq, priority);
    }

    /// Set the target CPUs for an interrupt.
    ///
    /// The target value is platform-dependent:
    /// - QEMU xlnx-zcu102 unified GIC: 0x10 (bit 4) for R5-0 (CPUs 0-3 = A53, 4-5 = R5)
    /// - Real ZynqMP hardware: 0x01 (bit 0) since RPU has its own GIC view
    ///
    /// Use `GIC_CPU_TARGET` from config_zynqmp to get the correct value.
    ///
    /// # Panics
    /// Panics if `irq > 1019` or `irq < 32` (SGI/PPI target registers are
    /// read-only on GICv2 -- these interrupts are CPU-private).
    pub fn set_target(&mut self, irq: u32, target: u8) {
        Self::validate_irq(irq);
        assert!(irq >= 32, "Cannot set target for SGI/PPI (IRQ < 32)");
        GicDistributor::<GICD_BASE>::set_target(irq, target);
    }

    /// Acknowledge an interrupt and return its ID.
    ///
    /// **Warning**: Do not use in deferred EOI mode. The assembly IRQ handler
    /// reads GICC_IAR and stores in `_tx_gic_iar`. Use `current_irq_id()` instead.
    #[deprecated(note = "Use current_irq_id() - assembly handles IAR read")]
    pub fn acknowledge() -> u32 {
        GicCpuInterface::<GICC_BASE>::acknowledge()
    }

    /// Signal end of interrupt processing.
    ///
    /// **Warning**: Do not use in deferred EOI mode. The assembly context_restore
    /// writes GICC_EOIR after CPSID. Calling this creates double-EOI bugs.
    #[deprecated(note = "Assembly handles EOI in deferred mode")]
    pub fn end_of_interrupt(irq: u32) {
        GicCpuInterface::<GICC_BASE>::end_of_interrupt(irq);
    }

    /// Clear a pending interrupt (static version for ISR use).
    ///
    /// For level-triggered interrupts, call this AFTER clearing the interrupt
    /// source but BEFORE end_of_interrupt() to prevent spurious re-triggering.
    ///
    /// # Panics
    /// Panics if `irq > 1019`.
    pub fn clear_pending_static(irq: u32) {
        Self::validate_irq(irq);
        GicDistributor::<GICD_BASE>::clear_pending(irq);
    }

    /// Check if an interrupt is pending.
    ///
    /// # Panics
    /// Panics if `irq > 1019`.
    pub fn is_pending(irq: u32) -> bool {
        Self::validate_irq(irq);
        GicDistributor::<GICD_BASE>::is_pending(irq)
    }
}

impl<const ADDR: usize> GicDistributor<ADDR> {
    /// Initialize the distributor.
    fn init() {
        #[cfg(feature = "zynqmp")]
        unsafe {
            let base = ADDR as *mut u32;

            // Disable distributor
            base.byte_add(gic::GICD_CTLR).write_volatile(0);

            let num_irqs = Self::get_num_irqs();

            // Clear all pending and enabled interrupts (stale state from warm reset)
            for i in (0..num_irqs).step_by(32) {
                let reg = (i / 32) as usize * 4;
                base.byte_add(gic::GICD_ICENABLER + reg)
                    .write_volatile(0xFFFF_FFFF);
                base.byte_add(gic::GICD_ICPENDR + reg)
                    .write_volatile(0xFFFF_FFFF);
            }

            // Set all priorities to 0xa0 (medium priority)
            for i in (32..num_irqs).step_by(4) {
                base.byte_add(gic::GICD_IPRIORITYR + i as usize)
                    .write_volatile(0xa0a0_a0a0);
            }

            // Set all SPIs to target the local R5 core.
            // The target mask is platform-dependent (see config_zynqmp::GIC_CPU_TARGET).
            let t = crate::config_zynqmp::GIC_CPU_TARGET as u32;
            let target_word = t | (t << 8) | (t << 16) | (t << 24);
            for i in (32..num_irqs).step_by(4) {
                base.byte_add(gic::GICD_ITARGETSR + i as usize)
                    .write_volatile(target_word);
            }

            // Configure all SPIs as level-triggered
            for i in (32..num_irqs).step_by(16) {
                base.byte_add(gic::GICD_ICFGR + ((i / 16) * 4) as usize)
                    .write_volatile(0);
            }

            // Ensure all configuration writes are visible before enabling
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));

            // Enable distributor
            base.byte_add(gic::GICD_CTLR)
                .write_volatile(gic::GICD_CTLR_ENABLE);

            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }
    }

    /// Get the number of supported interrupts.
    fn get_num_irqs() -> u32 {
        #[cfg(feature = "zynqmp")]
        unsafe {
            let base = ADDR as *const u32;
            let typer = base.byte_add(gic::GICD_TYPER).read_volatile();
            // ITLinesNumber field is bits [4:0]
            // Number of interrupts = 32 * (ITLinesNumber + 1)
            ((typer & 0x1F) + 1) * 32
        }
        #[cfg(not(feature = "zynqmp"))]
        0
    }

    /// Write a single-bit register (ISENABLER, ICENABLER, ICPENDR, etc.).
    ///
    /// These registers use bit-per-interrupt addressing: register offset = (irq/32)*4,
    /// bit position = irq % 32.
    #[cfg(feature = "zynqmp")]
    fn write_bit_reg(base_reg: usize, irq: u32) {
        unsafe {
            let base = ADDR as *mut u32;
            let reg_offset = (irq / 32) as usize * 4;
            let bit = 1u32 << (irq % 32);
            base.byte_add(base_reg + reg_offset).write_volatile(bit);
        }
    }

    /// Enable a specific interrupt.
    fn enable_interrupt(irq: u32) {
        #[cfg(feature = "zynqmp")]
        Self::write_bit_reg(gic::GICD_ISENABLER, irq);
    }

    /// Disable a specific interrupt.
    fn disable_interrupt(irq: u32) {
        #[cfg(feature = "zynqmp")]
        Self::write_bit_reg(gic::GICD_ICENABLER, irq);
    }

    /// Clear a pending interrupt.
    fn clear_pending(irq: u32) {
        #[cfg(feature = "zynqmp")]
        Self::write_bit_reg(gic::GICD_ICPENDR, irq);
    }

    /// Check if an interrupt is pending.
    fn is_pending(irq: u32) -> bool {
        #[cfg(feature = "zynqmp")]
        unsafe {
            let base = ADDR as *const u32;
            let reg_offset = (irq / 32) as usize * 4;
            let bit = 1u32 << (irq % 32);
            (base
                .byte_add(gic::GICD_ISPENDR + reg_offset)
                .read_volatile()
                & bit)
                != 0
        }
        #[cfg(not(feature = "zynqmp"))]
        {
            let _ = irq;
            false
        }
    }

    /// Set the priority of an interrupt.
    fn set_priority(irq: u32, priority: u8) {
        #[cfg(feature = "zynqmp")]
        unsafe {
            let reg_offset = (irq / 4) as usize * 4;
            let byte_offset = (irq % 4) as usize;
            let ptr = (ADDR + gic::GICD_IPRIORITYR + reg_offset) as *mut u8;
            ptr.add(byte_offset).write_volatile(priority);
        }
        #[cfg(not(feature = "zynqmp"))]
        {
            let _ = (irq, priority); // Silence unused warnings
        }
    }

    /// Set the target CPUs for an interrupt.
    fn set_target(irq: u32, target: u8) {
        #[cfg(feature = "zynqmp")]
        unsafe {
            let reg_offset = (irq / 4) as usize * 4;
            let byte_offset = (irq % 4) as usize;
            let ptr = (ADDR + gic::GICD_ITARGETSR + reg_offset) as *mut u8;
            ptr.add(byte_offset).write_volatile(target);
        }
    }
}

impl<const ADDR: usize> GicCpuInterface<ADDR> {
    /// Initialize the CPU interface.
    fn init() {
        #[cfg(feature = "zynqmp")]
        unsafe {
            let base = ADDR as *mut u32;

            // Set priority mask to allow all interrupts
            base.byte_add(gic::GICC_PMR).write_volatile(0xFF);

            // Set binary point to 0 (all priority bits used for preemption)
            base.byte_add(gic::GICC_BPR).write_volatile(0);

            // Enable CPU interface
            base.byte_add(gic::GICC_CTLR)
                .write_volatile(gic::GICC_CTLR_ENABLE);

            // Ensure enable is visible before IRQs are unmasked
            core::arch::asm!("dsb sy", "isb", options(nostack, preserves_flags));
        }
    }

    /// Acknowledge an interrupt and return its ID.
    ///
    /// Reading IAR acknowledges the highest priority pending interrupt.
    /// Returns 1023 if no interrupt is pending.
    fn acknowledge() -> u32 {
        #[cfg(feature = "zynqmp")]
        unsafe {
            let base = ADDR as *const u32;
            base.byte_add(gic::GICC_IAR).read_volatile()
        }
        #[cfg(not(feature = "zynqmp"))]
        1023
    }

    /// Signal end of interrupt processing.
    fn end_of_interrupt(irq: u32) {
        #[cfg(feature = "zynqmp")]
        unsafe {
            let base = ADDR as *mut u32;
            base.byte_add(gic::GICC_EOIR).write_volatile(irq);
        }
    }
}
