#!/usr/bin/env bash
# One-step installer for utk-steamvr (VIVE Ultimate Tracker for Linux SteamVR).
#
#   curl -fsSL https://github.com/xkeyC/utk-steamvr/releases/latest/download/install.sh | bash
#
# Installs into ~/.local/share/utk-steamvr (no root needed) and registers the SteamVR driver and
# the calibration tab in the SteamVR menu. HTC's tracker SDK library is downloaded from HTC's own
# repository on this machine, after you accept the VIVE Product EULA it is licensed under.
#
# Options:
#   --version vX.Y.Z   install that release instead of the latest
#   --accept-eula      accept the VIVE Product EULA without asking (or UTK_ACCEPT_EULA=1)
#   --uninstall        remove the registrations and ~/.local/share/utk-steamvr
#                      (calibration in ~/.config/utk-steamvr is kept)
set -euo pipefail

REPO=xkeyC/utk-steamvr
ASSET=utk-steamvr-linux-x86_64.tar.gz
SDK_COMMIT=8685112780734c2513c66d53b9eb104db47afcf9
SDK_SHA256=ac34cd0c5805b4ff55d8db805930e3f940f7f97285bd8155a0bd64f1a08950ce
SDK_URL="https://raw.githubusercontent.com/ViveRoboticsSoftware/vive-ultimate-tracker-sdk/$SDK_COMMIT/bin/linux/x64/libultimate-tracker.so"
EULA_URL=https://dl.vive.com/EULA/unified_eula_20230721_eng.pdf

dest="${XDG_DATA_HOME:-$HOME/.local/share}/utk-steamvr"
version=latest
accept=${UTK_ACCEPT_EULA:-}
action=install

while [ $# -gt 0 ]; do
  case $1 in
    --version) version=$2; shift ;;
    --accept-eula) accept=1 ;;
    --uninstall) action=uninstall ;;
    -h | --help) sed -n '2,15p' "$0" 2>/dev/null || true; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done

say() { printf '\033[1m==> %s\033[0m\n' "$*"; }
die() { printf '\033[31merror:\033[0m %s\n' "$*" >&2; exit 1; }

steamvr_running() { pgrep -x -u "$(id -u)" vrserver >/dev/null 2>&1; }

if [ "$action" = uninstall ]; then
  if [ -x "$dest/bin/utk-calibrate" ]; then
    "$dest/bin/utk-calibrate" --uninstall
  fi
  rm -rf "$dest"
  say "Removed $dest"
  exit 0
fi

[ "$(uname -s)" = Linux ] && [ "$(uname -m)" = x86_64 ] || die "only Linux x86_64 is supported"
for cmd in curl tar sha256sum; do
  command -v "$cmd" >/dev/null || die "$cmd is required"
done
[ -f "${XDG_CONFIG_HOME:-$HOME/.config}/openvr/openvrpaths.vrpath" ] ||
  die "SteamVR has not been set up for this user: install SteamVR from Steam and start it once"
steamvr_running && die "SteamVR is running: quit it first (it rewrites its settings on exit)"

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

if [ -n "${UTK_BASE_URL:-}" ]; then
  base=$UTK_BASE_URL # testing: a directory with the release assets (file:// works)
elif [ "$version" = latest ]; then
  base="https://github.com/$REPO/releases/latest/download"
else
  base="https://github.com/$REPO/releases/download/$version"
fi
say "Downloading utk-steamvr ($version)"
curl -fSL --retry 3 --progress-bar -o "$tmp/$ASSET" "$base/$ASSET"
curl -fsSL --retry 3 -o "$tmp/SHA256SUMS" "$base/SHA256SUMS"
(cd "$tmp" && grep " $ASSET\$" SHA256SUMS | sha256sum -c --quiet -) || die "checksum mismatch for $ASSET"

cat <<TXT

utk-steamvr needs HTC's VIVE Ultimate Tracker SDK library (libultimate-tracker.so).
It is not part of this project: it is downloaded from HTC's repository
  https://github.com/ViveRoboticsSoftware/vive-ultimate-tracker-sdk
and your use of it is governed by the VIVE Product EULA:
  $EULA_URL

TXT
if [ -z "$accept" ]; then
  { : </dev/tty; } 2>/dev/null || die "no terminal to ask on: rerun with --accept-eula (or UTK_ACCEPT_EULA=1) to accept the VIVE Product EULA"
  printf 'Do you accept the VIVE Product EULA? [y/N] '
  read -r answer </dev/tty
  case $answer in
    y | Y | yes | YES) ;;
    *) die "the EULA was not accepted; nothing was installed" ;;
  esac
fi
say "Downloading libultimate-tracker.so from HTC"
curl -fSL --retry 3 --progress-bar -o "$tmp/libultimate-tracker.so" "$SDK_URL"
echo "$SDK_SHA256  $tmp/libultimate-tracker.so" | sha256sum -c --quiet - ||
  die "libultimate-tracker.so does not match the expected checksum"

say "Installing to $dest"
tar -C "$tmp" -xzf "$tmp/$ASSET"
install -m755 "$tmp/libultimate-tracker.so" "$tmp/utk-steamvr/driver/utk/bin/linux64/"
mkdir -p "$dest"
# Replace the program files; keep what the app writes there (icon, input bindings).
rm -rf "$dest/bin" "$dest/driver" "$dest/licenses"
cp -r "$tmp/utk-steamvr/." "$dest/"

say "Registering with SteamVR"
"$dest/bin/utk-calibrate" --install

cat <<TXT

Done ($(cat "$dest/VERSION")). Next:
  1. Pair the trackers and build the tracking map in VIVE Hub once, then close VIVE Hub
     (it and the driver cannot use the tracker dongle at the same time).
  2. Start SteamVR, open the SteamVR menu and pick the "UTK" tab to calibrate.
Uninstall: curl -fsSL https://github.com/$REPO/releases/latest/download/install.sh | bash -s -- --uninstall
TXT
