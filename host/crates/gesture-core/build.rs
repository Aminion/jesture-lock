//! Builds `core/` (the core.h contract plus its implementation) with its own
//! CMakeLists.txt so the host and MCU share one source list and one set of compile
//! options. `GESTURE_CORE_IMPL` in the environment picks the implementation (a directory
//! under `core/`, see `core/CMakeLists.txt`), `cpp` when unset. Always passed: the CMake
//! cache in OUT_DIR would otherwise keep the last value after the variable is unset.

use std::path::PathBuf;

fn main() {
    let core_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../core");
    println!("cargo:rerun-if-changed={}", core_dir.display());
    println!(
        "cargo:rerun-if-changed={}",
        core_dir.join("../cmake/gesture_options.cmake").display()
    );

    println!("cargo:rerun-if-env-changed=GESTURE_CORE_IMPL");

    let implementation = std::env::var("GESTURE_CORE_IMPL").unwrap_or_else(|_| "cpp".into());
    let dst = cmake::Config::new(&core_dir)
        .define("GESTURE_CORE_IMPL", implementation)
        .build();
    println!("cargo:rustc-link-search=native={}/lib", dst.display());
    println!("cargo:rustc-link-lib=static=gesture_core");
}
