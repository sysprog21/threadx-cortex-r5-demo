//! Build Script for threadx-sys
//!
//! By default, uses pre-generated bindings from src/bindings.rs.
//! Enable the `bindgen` feature to regenerate from ThreadX headers
//! (requires libclang).
//!
//! After regenerating, copy the output to src/bindings.rs:
//!   cp target/armv7r-none-eabihf/release/build/threadx-sys-*/out/bindings.rs threadx-sys/src/bindings.rs

use std::env;
use std::path::PathBuf;

fn main() {
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap()).join("bindings.rs");

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=wrapper.h");
    println!("cargo:rerun-if-changed=include");

    #[cfg(feature = "bindgen")]
    {
        let threadx_common_inc = PathBuf::from("../threadx/common/inc");
        let threadx_port_inc = PathBuf::from("../threadx/ports/cortex_r5/gnu/inc");
        println!("cargo:rerun-if-changed={}", threadx_common_inc.display());
        println!("cargo:rerun-if-changed={}", threadx_port_inc.display());
        generate_bindings(&out_path);
    }

    #[cfg(not(feature = "bindgen"))]
    {
        let pregenerated = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap())
            .join("src")
            .join("bindings.rs");
        println!("cargo:rerun-if-changed={}", pregenerated.display());
        std::fs::copy(&pregenerated, &out_path).expect(
            "Failed to copy pre-generated bindings. \
             Enable the 'bindgen' feature to regenerate them.",
        );
    }
}

#[cfg(feature = "bindgen")]
fn generate_bindings(out_path: &std::path::Path) {
    let threadx_path = PathBuf::from("../threadx");

    let bindings = bindgen::Builder::default()
        .header("wrapper.h")
        .clang_arg(format!("-I{}", threadx_path.join("common/inc").display()))
        .clang_arg(format!(
            "-I{}",
            threadx_path.join("ports/cortex_r5/gnu/inc").display()
        ))
        .clang_arg("-I./include")
        .clang_arg("-nostdinc")
        .clang_arg("--target=arm")
        .clang_arg("-mthumb")
        .clang_arg("-mcpu=cortex-r5")
        .clang_arg("-mfloat-abi=hard")
        .use_core()
        .allowlist_function("tx_.*")
        .allowlist_function("_tx_.*")
        .allowlist_type("TX_.*")
        .allowlist_var("TX_.*")
        .allowlist_var("TX_AUTO_START")
        .formatter(bindgen::Formatter::Rustfmt)
        .generate()
        .expect("Unable to generate bindings");

    bindings
        .write_to_file(out_path)
        .expect("Couldn't write bindgen output");

    eprintln!(
        "Bindings generated at {:?}. Copy to threadx-sys/src/bindings.rs to update the pre-generated file.",
        out_path
    );
}
