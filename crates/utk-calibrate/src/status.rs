//! Status published by the SteamVR driver in `$XDG_STATE_HOME/utk-steamvr/status.json`
//! (rewritten twice a second): tracker service state and per-tracker sync state.

use std::path::PathBuf;

use serde::Deserialize;

#[derive(Debug, Clone, Deserialize)]
pub struct Status {
    pub updated_unix_ms: u64,
    /// starting | running | vive_hub_running | start_failed | stopped
    pub server: String,
    #[serde(default)]
    pub trackers: Vec<Tracker>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Tracker {
    pub serial: String,
    pub present: bool,
    /// pairing | paired | connecting | connected | initialized | syncing | building_map | ready
    pub state: String,
    pub host: bool,
    pub rotation_only: bool,
    pub setup_required: bool,
    pub battery: u8,
    pub rate_hz: f64,
    pub latency_ms: f64,
}

fn path() -> Option<PathBuf> {
    std::env::var_os("XDG_STATE_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".local/state")))
        .map(|d| d.join("utk-steamvr/status.json"))
}

pub fn read() -> Option<Status> {
    serde_json::from_str(&std::fs::read_to_string(path()?).ok()?).ok()
}

impl Status {
    /// The driver rewrites the file twice a second; older than this means it is not running.
    pub fn is_fresh(&self) -> bool {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_millis() as u64);
        now.saturating_sub(self.updated_unix_ms) < 3000
    }
}

impl Tracker {
    pub fn state_label(&self) -> &'static str {
        match self.state.as_str() {
            "ready" if self.rotation_only => "Lost tracking (rotation only)",
            "ready" => "Ready",
            "syncing" if self.setup_required => "No map: build one in VIVE Hub",
            "syncing" => "Syncing map",
            "building_map" => "Building map",
            "initialized" => "Initialized",
            "connected" => "Connected",
            "connecting" => "Connecting (press its button to wake it)",
            "paired" => "Paired",
            "pairing" => "Pairing",
            _ => "Unknown",
        }
    }
}
