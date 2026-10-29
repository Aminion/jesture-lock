//! Builds `core/` with its own CMakeLists.txt so the host and MCU share one
//! source list and one set of compile options.

use std::path::PathBuf;

fn main() {
    let core_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../core");
    println!("cargo:rerun-if-changed={}", core_dir.display());
    println!(
        "cargo:rerun-if-changed={}",
        core_dir.join("../cmake/gesture_options.cmake").display()
    );

    let dst = cmake::Config::new(&core_dir).build();
    println!("cargo:rustc-link-search=native={}/lib", dst.display());
    println!("cargo:rustc-link-lib=static=gesture_core");
}
