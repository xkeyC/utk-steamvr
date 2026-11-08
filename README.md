# UTK for Linux SteamVR

[中文](README.zh-CN.md)

Use VIVE Ultimate Trackers (UTK) as SteamVR trackers on Linux, with headsets that have no
Lighthouse tracking of their own, such as a Quest 3 streaming over Steam Link.

On Windows, HTC's setup is VIVE Hub + a SteamVR driver + "VIVE Space Calibrator". On Linux, HTC
ships VIVE Hub for Linux (pairing, map building) and the VIVE Ultimate Tracker Linux SDK, but no
SteamVR driver and no calibration. This project provides both:

- **SteamVR driver**: runs HTC's tracker SDK inside SteamVR. About 125 Hz per tracker with 2–4 ms
  of data latency; the trackers show up as generic SteamVR trackers (assign body roles under
  "Manage Trackers").
- **In-VR calibration**: a "UTK Calibration" tab in the SteamVR menu. Hold a controller and any
  tracker together and draw figure eights; the tracker is detected from the motion, no buttons
  needed.

> This is an unofficial community project. It is not affiliated with or endorsed by HTC or Valve.
> VIVE and HTC are trademarks of HTC Corporation.

## Install

Requirements: Linux x86_64, glibc 2.35 or newer (Ubuntu 22.04, Debian 12, SteamOS 3, Arch,
Fedora…), SteamVR from the native Steam package, started at least once. Flatpak Steam is not
supported yet.

Quit SteamVR, then run:

```sh
curl -fsSL https://github.com/xkeyC/utk-steamvr/releases/latest/download/install.sh | bash
```

The installer:

1. downloads the latest release and checks its SHA-256;
2. shows the VIVE Product EULA and, once you accept it, downloads `libultimate-tracker.so` from
   [HTC's SDK repository](https://github.com/ViveRoboticsSoftware/vive-ultimate-tracker-sdk)
   (pinned commit, hash-checked);
3. installs everything to `~/.local/share/utk-steamvr` (no root);
4. registers the driver in `~/.config/openvr/openvrpaths.vrpath`, turns on
   `steamvr/activateMultipleDrivers` and adds the calibration tab to SteamVR. Every edited file is
   backed up once as `*.utk-backup`.

Run the same command again to update. Options go after `bash -s --`: `--version v0.1.0`,
`--accept-eula` (non-interactive), `--uninstall` (keeps your calibration in
`~/.config/utk-steamvr`).

`activateMultipleDrivers` matters: with it off, SteamVR only loads the driver that provides the
headset (Steam Link's vrlink, ALVR, …) and never loads this one.

## Use

### Once: pair and build the map in VIVE Hub

Pair the trackers and build the tracking map in VIVE Hub ("Start setup"). The map is stored in the
trackers; you only redo this when the room changes a lot.

**Then close VIVE Hub.** VIVE Hub and the SteamVR driver both need exclusive access to the
tracker dongle. While VIVE Hub runs, the driver does not start the tracker service and the
calibration tab tells you to close it; once it is closed the driver takes over on its own, no
SteamVR restart needed.

### Calibrate in VR

Open the SteamVR menu and pick the "UTK" icon in the bottom bar:

1. The panel shows the tracker service and each tracker's state (ready, syncing the map, map
   needed, …), battery, update rate and latency.
2. Click "Start calibration", or press A while the menu is closed.
3. Hold any controller firmly together with any tracker and draw large figure eights while
   turning your wrist. The pair is detected automatically; about 25 s of good motion with enough
   rotation is needed.
4. If the fit is within 20 mm and 3°, it is saved to `~/.config/utk-steamvr/calibration.json` and
   the driver applies it within a second; otherwise you are asked to try again.

With the menu closed: A starts, Y shows the current calibration, B cancels. While the menu is open
SteamVR keeps controller buttons to itself, so use the laser pointer there.

Recalibrate after rebuilding the map, or after the headset's guardian/boundary has been reset or
re-centred.

## Build from source

With Nix:

```sh
nix develop
cargo build && cargo test
./scripts/install-driver.sh        # builds and installs into ~/.local/share/utk-steamvr
```

Without Nix you need Rust (stable), clang/libclang, pkg-config and the EGL/GL development
headers; `scripts/ci-deps.sh` downloads `openvr_driver.h` and HTC's SDK library and prints the
environment variables the build needs. See `.github/workflows/ci.yml` for the full recipe;
`scripts/package.sh` builds the release archive.

Tools:

```sh
./target/release/utk-calibrate --snapshot panel.png   # render the panel to a PNG, no SteamVR needed
./target/release/utk status --watch                    # driver status in the terminal (status.json)
./target/release/utk record -d 30                      # record SteamVR poses of controllers, HMD, utk trackers
```

On NixOS the release binaries need `programs.nix-ld.enable` for the installer's last step; the
Nix build above works without it.

## Layout

```
crates/utk-driver/     SteamVR driver: C++ OpenVR driver interfaces + HTC SDK, Rust for poses, calibration, status
crates/utk-driver/sdk/ HTC SDK header (Apache-2.0)
crates/utk-calib/      calibration math (pair detection, yaw-only Kabsch, joint Gauss–Newton refinement)
crates/utk-calibrate/  in-VR calibration: SteamVR menu tab + floating panel (egui on headless EGL), installer
crates/utk-cli/        `utk` command line: status / record
scripts/               installers, CI dependency fetcher, packaging
flake.nix              development shell; fetches HTC's SDK library from HTC's GitHub
```

## Runtime files

| File | Contents |
|---|---|
| `~/.config/utk-steamvr/calibration.json` | calibration: tracker map → SteamVR raw tracking space |
| `~/.local/state/utk-steamvr/status.json` | written by the driver every 0.5 s: service and per-tracker state |
| `~/.local/state/utk-steamvr/calibrate.log` | calibration app log |
| `vrserver.txt` (Steam's logs directory) | driver log, lines start with `utk:` |

## Notes

- HTC SDK callbacks: about 125 Hz per tracker, one sample at a time (8 ms apart),
  `GetTimestamp() - Timestamp_us` is 2–4 ms. Angular velocity is provided, linear velocity is 0.
  Poses are in the Isaac Sim frame (X forward, Y left, Z up); the driver converts them to OpenVR
  (X right, Y up, Z back). A `PoseStable` flag marks a resting tracker, whose pose the driver then
  holds still.
- VIVE Hub's own `tracker_server` sends poses in batches every 50 ms (the dongle itself reports
  every ~3 ms). Reading poses through it costs 25 ms+ of latency and visible steps; the SDK path
  avoids both.
- The tracker map is gravity-aligned; its origin is at the height the host tracker was held at
  while building the map.
- SteamVR's IPC uses abstract sockets, which are per network namespace; Steam and pressure-vessel
  have private `/tmp` directories.
- Calibration on a recorded session: the right pair correlates at 0.99 vs ≤ 0.24 for the others;
  position residual RMS 8.8 mm, median orientation residual 1.1°.

## License

Licensed under either of [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at your option.

`crates/utk-driver/sdk/ultimate_tracker.h` is HTC's, under Apache-2.0. HTC's SDK library
(`libultimate-tracker.so`) is governed by the
[VIVE Product EULA](https://dl.vive.com/EULA/unified_eula_20230721_eng.pdf), which does not allow
redistributing it, so it is neither in this repository nor in the release archives; the installer
downloads it from HTC on your machine. Release archives also contain `libopenvr_api.so` (Valve,
BSD-3-Clause) and fonts embedded by egui; their licenses are in `licenses/`.
