{
  description = "VIVE Ultimate Tracker for Linux SteamVR";

  # Same nixpkgs revision as ~/Nix, so dependencies come from the local store.
  inputs.nixpkgs.url = "github:NixOS/nixpkgs/1d060086d60cb76a40cf37109f979d88a5bd6396";

  outputs = { nixpkgs, ... }:
    let
      pkgs = nixpkgs.legacyPackages.x86_64-linux;
      inherit (pkgs) lib;
      # HTC VIVE Ultimate Tracker Linux SDK (beta). The library is under HTC's EULA, so it is
      # downloaded (hash-checked) instead of being stored in the repository.
      vutSdk = pkgs.runCommand "vive-ultimate-tracker-sdk-0.0.1" {
        src = pkgs.fetchurl {
          url = "https://raw.githubusercontent.com/ViveRoboticsSoftware/vive-ultimate-tracker-sdk/8685112780734c2513c66d53b9eb104db47afcf9/bin/linux/x64/libultimate-tracker.so";
          hash = "sha256-rDTNDFgFtP9V2NuAWTDj+UD3+XKFvYFVoL1k8aCJUM4=";
        };
      } "install -Dm755 $src $out/lib/libultimate-tracker.so";
    in
    {
      devShells.x86_64-linux.default = pkgs.mkShell {
        nativeBuildInputs = with pkgs; [
          cargo
          rustc
          clippy
          rustfmt
          rust-analyzer
          pkg-config
          # ovr_overlay generates OpenVR bindings with autocxx, which needs libclang and header flags.
          rustPlatform.bindgenHook
        ];
        # egl.pc for khronos-egl (utk-calibrate renders its panel with headless EGL + OpenGL).
        buildInputs = [ pkgs.libglvnd ];
        # ovr_overlay links against its bundled libopenvr_api.so without an rpath; use nixpkgs' openvr
        # at runtime instead.
        RUSTFLAGS = "-C link-arg=-Wl,-rpath,${lib.makeLibraryPath [ pkgs.openvr ]}";
        # SteamVR's vrclient.so needs libGL / libEGL / libuuid, which are not on the NixOS host's
        # default paths (dlopen fails as VRInitError_Init_VRClientDLLNotFound). crates/utk-cli/build.rs
        # links them into the `utk` binary only, so vrclient.so reuses them by soname. The driver
        # runs inside vrserver and must not pull them in.
        UTK_VRCLIENT_LIB_DIRS = lib.makeLibraryPath [ pkgs.libglvnd pkgs.util-linux.lib ];
        # The driver links HTC's direct tracker SDK (crates/utk-driver/sdk has the header).
        VUT_SDK_LIB_DIR = "${vutSdk}/lib";
        # The driver's C++ layer needs openvr_driver.h.
        OPENVR_INCLUDE = "${pkgs.openvr}/include/openvr";
        # mkShell points $out at ./outputs/out; keep it out of the rpath.
        NIX_NO_SELF_RPATH = 1;
      };
    };
}
