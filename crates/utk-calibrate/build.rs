fn main() {
    // Release archives ship libopenvr_api.so next to the binaries.
    println!("cargo:rustc-link-arg-bins=-Wl,-rpath,$ORIGIN");

    // NixOS only (flake.nix sets UTK_VRCLIENT_LIB_DIRS): SteamVR's vrclient.so needs libGL /
    // libEGL / libuuid, which are not on the host's default library paths there, so link them
    // into our binaries and vrclient.so reuses them by soname. Other distributions have them.
    println!("cargo:rerun-if-env-changed=UTK_VRCLIENT_LIB_DIRS");
    let Ok(dirs) = std::env::var("UTK_VRCLIENT_LIB_DIRS") else {
        return;
    };
    for dir in dirs.split(':').filter(|d| !d.is_empty()) {
        println!("cargo:rustc-link-search=native={dir}");
        println!("cargo:rustc-link-arg-bins=-Wl,-rpath,{dir}");
    }
    println!(
        "cargo:rustc-link-arg-bins=-Wl,--push-state,--no-as-needed,-lGL,-lEGL,-luuid,--pop-state"
    );
}
