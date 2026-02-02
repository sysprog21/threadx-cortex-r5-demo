//! Compile-fail tests for type safety guarantees.
//!
//! These tests verify that the safe module correctly enforces Rust's type
//! system constraints at compile time, including:
//!
//! - Lifetime constraints (guards can't outlive resources)
//! - Send/Sync bounds (!Send guards prevent cross-thread usage)
//! - Pinning requirements (contexts must be pinned)
//!
//! Run with: `cargo test --features host-test`

#[test]
fn compile_fail() {
    let t = trybuild::TestCases::new();
    t.compile_fail("tests/ui/*.rs");
}
