#!/usr/bin/env bash
# Development install: builds from this checkout and installs into ~/.local/share/utk-steamvr the
# same way the release installer (scripts/install.sh) does, then registers it with SteamVR.
# Quit SteamVR first the first time (registration edits SteamVR's settings, which a running
# vrserver overwrites); later updates only replace files and need a SteamVR restart.
#
# The driver lives under $HOME because Steam's container cannot see every mount point.
# `--uninstall` removes the registrations.
set -euo pipefail

root=$(cd "$(dirname "$0")/.." && pwd)
dest="${XDG_DATA_HOME:-$HOME/.local/share}/utk-steamvr"

if [ "${1:-}" = "--uninstall" ]; then
  exec "$dest/bin/utk-calibrate" --uninstall
fi

if [ -z "${OPENVR_INCLUDE:-}" ]; then
  exec nix develop "$root" -c "$0" "$@"
fi

cargo build --release -p utk-driver -p utk-calibrate -p utk-cli --manifest-path "$root/Cargo.toml"
release=$root/target/release
driver=$dest/driver/utk
rm -rf "$driver"
mkdir -p "$driver/bin/linux64" "$dest/bin"
cp -r "$root/crates/utk-driver/driver/utk/." "$driver/"
install -m755 "$release/libdriver_utk.so" "$driver/bin/linux64/driver_utk.so"
# HTC's tracker SDK: VIVE Product EULA, so it is not in this repository; flake.nix downloads it.
install -m755 "$VUT_SDK_LIB_DIR/libultimate-tracker.so" "$driver/bin/linux64/libultimate-tracker.so"
install -m755 "$release/utk-calibrate" "$release/utk" "$dest/bin/"
echo "installed to $dest"
"$dest/bin/utk-calibrate" --install
echo "restart SteamVR to load the new build; driver log lines start with 'utk:' in vrserver.txt"
