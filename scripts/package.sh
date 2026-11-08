#!/usr/bin/env bash
# Packs a release build into dist/utk-steamvr-linux-x86_64.tar.gz (+ install.sh, SHA256SUMS).
# Run after `cargo build --release --locked` with the environment from scripts/ci-deps.sh.
#
# Archive layout (extracted to ~/.local/share/utk-steamvr by install.sh):
#   bin/utk-calibrate, bin/utk, bin/libopenvr_api.so
#   driver/utk/...            SteamVR driver; install.sh adds bin/linux64/libultimate-tracker.so
#   licenses/                 this project's and third-party licenses
set -euo pipefail

root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"
version=${1:-$(git describe --tags --always --dirty 2>/dev/null || echo dev)}
dist=$root/dist
stage=$dist/stage/utk-steamvr
rm -rf "$dist"
mkdir -p "$stage/bin" "$stage/driver" "$stage/licenses"

release=target/release
install -m755 "$release/utk-calibrate" "$release/utk" "$stage/bin/"
openvr_api=$(find "$release/build" -path '*ovr_overlay*' -name libopenvr_api.so -print -quit)
[ -n "$openvr_api" ] || { echo "libopenvr_api.so not found under $release/build" >&2; exit 1; }
install -m755 "$openvr_api" "$stage/bin/"

cp -r crates/utk-driver/driver/utk "$stage/driver/"
mkdir -p "$stage/driver/utk/bin/linux64"
install -m755 "$release/libdriver_utk.so" "$stage/driver/utk/bin/linux64/driver_utk.so"

# Licenses: ours, the SDK header's, OpenVR's (libopenvr_api.so) and egui's bundled fonts.
cp LICENSE-MIT LICENSE-APACHE "$stage/licenses/"
meta=$(cargo metadata --format-version 1 --locked)
dir_of() { echo "$meta" | jq -r --arg n "$1" '.packages[] | select(.name == $n) | .manifest_path' | head -1 | xargs dirname; }
fonts=$(dir_of epaint_default_fonts)/fonts
mkdir -p "$stage/licenses/egui-fonts"
cp "$fonts"/*.txt "$stage/licenses/egui-fonts/"
cp "$(dir_of ovr_overlay_sys)/openvr/LICENSE" "$stage/licenses/OpenVR-LICENSE"
cat > "$stage/licenses/README" <<TXT
utk-steamvr $version is licensed under MIT OR Apache-2.0 (LICENSE-MIT, LICENSE-APACHE).
crates/utk-driver/sdk/ultimate_tracker.h: Copyright (c) 2026 HTC Corporation, Apache-2.0.
bin/libopenvr_api.so: Valve OpenVR, BSD-3-Clause (OpenVR-LICENSE).
Fonts embedded by egui: see egui-fonts/.
Rust dependencies are MIT/Apache-2.0/Zlib/BSD/ISC licensed; see Cargo.lock for the exact set.
libultimate-tracker.so is not included: install.sh downloads it from HTC under the VIVE Product EULA.
TXT
echo "$version" > "$stage/VERSION"
cp README.md README.zh-CN.md "$stage/"

tar -C "$dist/stage" --owner=0 --group=0 -czf "$dist/utk-steamvr-linux-x86_64.tar.gz" utk-steamvr
install -m755 scripts/install.sh "$dist/install.sh"
(cd "$dist" && sha256sum utk-steamvr-linux-x86_64.tar.gz install.sh > SHA256SUMS)
rm -rf "$dist/stage"
ls -l "$dist"
