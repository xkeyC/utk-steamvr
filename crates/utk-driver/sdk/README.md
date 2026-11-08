`ultimate_tracker.h` comes from HTC's VIVE Ultimate Tracker Linux SDK
(https://github.com/ViveRoboticsSoftware/vive-ultimate-tracker-sdk, commit
8685112780734c2513c66d53b9eb104db47afcf9) and is Apache-2.0 per its SPDX header.

The library itself (`libultimate-tracker.so`) is under HTC's VIVE Product EULA and is not
stored here: flake.nix downloads it from the same commit (hash-checked) and exposes it as
`VUT_SDK_LIB_DIR`; scripts/install-driver.sh copies it next to `driver_utk.so`.
