#!/usr/bin/env bash
# Build inputs for a non-Nix build (CI): OpenVR's openvr_driver.h and HTC's tracker SDK library,
# both pinned and hash-checked. The SDK library is only linked against; it is never packaged
# (VIVE Product EULA), install.sh downloads it on the user's machine instead.
#
# Usage: scripts/ci-deps.sh [dir]   (default .deps). Prints the environment to use; on GitHub
# Actions it is also appended to $GITHUB_ENV.
set -euo pipefail

root=$(cd "$(dirname "$0")/.." && pwd)
deps=$(mkdir -p "${1:-$root/.deps}" && cd "${1:-$root/.deps}" && pwd)
# The SDK pin lives in install.sh, which users run on its own.
eval "$(grep -E '^SDK_(COMMIT|SHA256)=' "$root/scripts/install.sh")"
OPENVR_TAG=v2.15.6
OPENVR_DRIVER_H_SHA256=1036efe998d63e82d1d3db2b32a2f58df4a8eeaf5280f50aaf28220ff60a40ab

fetch() { # url sha256 out
  curl -fsSL --retry 3 -o "$3" "$1"
  echo "$2  $3" | sha256sum -c --quiet -
}

mkdir -p "$deps/openvr" "$deps/sdk"
fetch "https://raw.githubusercontent.com/ValveSoftware/openvr/$OPENVR_TAG/headers/openvr_driver.h" \
  "$OPENVR_DRIVER_H_SHA256" "$deps/openvr/openvr_driver.h"
fetch "https://raw.githubusercontent.com/ViveRoboticsSoftware/vive-ultimate-tracker-sdk/$SDK_COMMIT/bin/linux/x64/libultimate-tracker.so" \
  "$SDK_SHA256" "$deps/sdk/libultimate-tracker.so"

env=$(cat <<ENV
OPENVR_INCLUDE=$deps/openvr
VUT_SDK_LIB_DIR=$deps/sdk
LD_LIBRARY_PATH=$deps/sdk${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
ENV
)
echo "$env"
[ -n "${GITHUB_ENV:-}" ] && echo "$env" >> "$GITHUB_ENV"
exit 0
