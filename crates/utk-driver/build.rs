// Builds the C++ layer that implements the OpenVR driver interfaces. Headers come from
// OPENVR_INCLUDE (set by flake.nix to nixpkgs' openvr include/openvr).
fn main() {
    let include = std::env::var("OPENVR_INCLUDE")
        .expect("OPENVR_INCLUDE is not set (run inside `nix develop`, or see scripts/ci-deps.sh)");
    println!("cargo:rerun-if-changed=src/driver.cpp");
    println!("cargo:rerun-if-env-changed=OPENVR_INCLUDE");
    println!("cargo:rerun-if-changed=sdk/ultimate_tracker.h");
    println!("cargo:rerun-if-env-changed=VUT_SDK_LIB_DIR");

    cc::Build::new()
        .cpp(true)
        .std("c++17")
        .file("src/driver.cpp")
        // Treat as a system header so openvr_driver.h's own warnings stay quiet.
        .flag(format!("-isystem{include}"))
        .opt_level(2)
        .flag("-fvisibility=hidden")
        .warnings(true)
        .compile("utk_driver_cpp");
    // HTC's direct tracker SDK, after the C++ archive that uses it (the linker drops libraries
    // nothing has referenced yet). At runtime it is loaded from next to driver_utk.so ($ORIGIN),
    // where scripts/install-driver.sh puts it.
    let sdk = std::env::var("VUT_SDK_LIB_DIR")
        .expect("VUT_SDK_LIB_DIR is not set (run inside `nix develop`, or see scripts/ci-deps.sh)");
    println!("cargo:rustc-link-search=native={sdk}");
    println!("cargo:rustc-link-lib=dylib=ultimate-tracker");
    println!("cargo:rustc-cdylib-link-arg=-Wl,-rpath,$ORIGIN");
    // HmdDriverFactory itself is exported from Rust (lib.rs): a cdylib's version script only
    // exports Rust #[no_mangle] symbols, so a C++ definition would stay hidden.
}
