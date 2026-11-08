//! Rust layer of the SteamVR driver.
//!
//! The C++ layer (driver.cpp) runs HTC's direct tracker SDK inside vrserver: the SDK owns the
//! dongle and calls back for every pose (~125 Hz per tracker, 2–4 ms old). Those callbacks land
//! here (`utk_sdk_*`); this layer converts poses to the OpenVR frame, estimates velocities, holds
//! still trackers steady, applies the calibration, and publishes a status file for the in-VR
//! calibration panel.
//!
//! - Calibration: `$XDG_CONFIG_HOME/utk-steamvr/calibration.json` (default `~/.config/…`), see
//!   [`Calibration`]; reloaded within a second of changing, identity without it.
//! - Status: `$XDG_STATE_HOME/utk-steamvr/status.json` (default `~/.local/state/…`), rewritten
//!   twice a second.
//! - VIVE Hub's tracker_server also wants the dongle, so the SDK is only started while VIVE Hub
//!   is not running ([`utk_vive_hub_running`]).

use std::collections::{BTreeMap, VecDeque};
use std::ffi::{CStr, CString, c_char};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::thread::JoinHandle;
use std::time::{Duration, SystemTime};

use serde::{Deserialize, Serialize};

/// Must match UtkPose in driver.cpp.
#[repr(C)]
pub struct UtkPose {
    connected: bool,
    valid: bool,
    age_s: f64,
    pos: [f64; 3],
    rot: [f64; 4],
    vel: [f64; 3],
    ang_vel: [f64; 3],
    world_rot: [f64; 4],
    world_pos: [f64; 3],
}

/// Must match UtkSdkSample in driver.cpp (a copy of vive::TrackerData plus the SDK clock).
#[repr(C)]
pub struct UtkSdkSample {
    location: [f32; 3],
    rotation: [f32; 4],
    angular_velocity: [f32; 3],
    timestamp_us: i64,
    sdk_now_us: i64,
    id: u32,
    flags: u32,
    state: u8,
    battery: u8,
}

type LogFn = extern "C" fn(*const c_char);

/// Calibration: p_steamvr = R(rotation) · p_map + translation, rotation as (x, y, z, w).
#[derive(Debug, Clone, Deserialize)]
struct Calibration {
    rotation: [f64; 4],
    translation: [f64; 3],
    #[serde(default)]
    tracker_serial: Option<String>,
}

impl Default for Calibration {
    fn default() -> Self {
        Self {
            rotation: [0.0, 0.0, 0.0, 1.0],
            translation: [0.0; 3],
            tracker_serial: None,
        }
    }
}

// vive::TrackerFlag_* (sdk/ultimate_tracker.h)
const FLAG_POSE_STABLE: u32 = 1 << 4;
const FLAG_ROTATION_ONLY: u32 = 1 << 5;
const FLAG_HOST: u32 = 1 << 6;
const FLAG_SETUP_REQUIRED: u32 = 1 << 7;
/// vive::TrackerState::Ready
const STATE_READY: u8 = 8;

/// Server state reported by the C++ supervisor (utk_set_server_state).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
enum ServerState {
    Starting,
    Running,
    ViveHubRunning,
    StartFailed,
    Stopped,
}

impl ServerState {
    fn from_raw(v: i32) -> Self {
        match v {
            1 => Self::Running,
            2 => Self::ViveHubRunning,
            3 => Self::StartFailed,
            4 => Self::Stopped,
            _ => Self::Starting,
        }
    }
}

struct Sample {
    /// Estimated capture time, CLOCK_MONOTONIC ns.
    t_ns: i64,
    pos: [f64; 3],
    rot: [f64; 4], // w, x, y, z (OpenVR frame)
}

#[derive(Default)]
struct Tracker {
    serial: String,
    present: bool,
    state: u8,
    flags: u32,
    battery: u8,
    history: VecDeque<Sample>,
    /// Pose frozen while the SDK reports the tracker as completely still (removes jitter).
    held: Option<([f64; 3], [f64; 4])>,
    latency_ms: f64,
    /// Updates counted for the status file's rate.
    updates: u32,
    rate_hz: f64,
}

#[derive(Default)]
struct Shared {
    trackers: BTreeMap<u32, Tracker>,
    /// Serials seen but not yet registered by the C++ layer.
    pending: VecDeque<String>,
    known: std::collections::HashSet<String>,
    calibration: Calibration,
    calibration_ok: bool,
}

struct Driver {
    shared: Arc<Mutex<Shared>>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

static DRIVER: Mutex<Option<Driver>> = Mutex::new(None);
static LOG: OnceLock<LogFn> = OnceLock::new();
static SERVER_STATE: AtomicI32 = AtomicI32::new(0);

fn log(msg: impl AsRef<str>) {
    if let Some(f) = LOG.get()
        && let Ok(c) = CString::new(format!("{}\n", msg.as_ref()))
    {
        f(c.as_ptr());
    }
}

/// Velocity is fitted over this much history (~5 samples at 125 Hz).
const VELOCITY_WINDOW_NS: i64 = 40_000_000;
const HISTORY_NS: i64 = 200_000_000;
/// Report the pose as invalid when nothing newer arrived within this time.
const STALE_NS: i64 = 250_000_000;

fn monotonic_ns() -> i64 {
    let t = nix::time::clock_gettime(nix::time::ClockId::CLOCK_MONOTONIC).expect("CLOCK_MONOTONIC");
    t.tv_sec() * 1_000_000_000 + t.tv_nsec()
}

fn home_dir(var: &str, fallback: &str) -> Option<PathBuf> {
    std::env::var_os(var)
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(fallback)))
}

fn calibration_path() -> Option<PathBuf> {
    home_dir("XDG_CONFIG_HOME", ".config").map(|d| d.join("utk-steamvr/calibration.json"))
}

fn status_path() -> Option<PathBuf> {
    home_dir("XDG_STATE_HOME", ".local/state").map(|d| d.join("utk-steamvr/status.json"))
}

/// Background thread: calibration reload, rate bookkeeping and the status file.
fn housekeeping(shared: Arc<Mutex<Shared>>, stop: Arc<AtomicBool>) {
    let calib_path = calibration_path();
    let mut calib_mtime: Option<SystemTime> = None;
    let mut tick = 0u64;
    while !stop.load(Ordering::SeqCst) {
        reload_calibration(&shared, calib_path.as_ref(), &mut calib_mtime);
        {
            let mut s = shared.lock().unwrap();
            for t in s.trackers.values_mut() {
                t.rate_hz = f64::from(t.updates) / 0.5;
                t.updates = 0;
            }
        }
        write_status(&shared);
        tick += 1;
        let _ = tick;
        std::thread::sleep(Duration::from_millis(500));
    }
}

#[derive(Serialize)]
struct StatusTracker<'a> {
    id: u32,
    serial: &'a str,
    present: bool,
    state: &'static str,
    host: bool,
    rotation_only: bool,
    setup_required: bool,
    stable: bool,
    battery: u8,
    rate_hz: f64,
    latency_ms: f64,
    age_ms: Option<f64>,
}

fn state_name(s: u8) -> &'static str {
    match s {
        1 => "pairing",
        2 => "paired",
        3 => "connecting",
        4 => "connected",
        5 => "initialized",
        6 => "syncing",
        7 => "building_map",
        8 => "ready",
        _ => "unknown",
    }
}

fn write_status(shared: &Mutex<Shared>) {
    let Some(path) = status_path() else { return };
    let now = monotonic_ns();
    let json = {
        let s = shared.lock().unwrap();
        let trackers: Vec<StatusTracker> = s
            .trackers
            .iter()
            .map(|(id, t)| StatusTracker {
                id: *id,
                serial: &t.serial,
                present: t.present,
                state: state_name(t.state),
                host: t.flags & FLAG_HOST != 0,
                rotation_only: t.flags & FLAG_ROTATION_ONLY != 0,
                setup_required: t.flags & FLAG_SETUP_REQUIRED != 0,
                stable: t.flags & FLAG_POSE_STABLE != 0,
                battery: t.battery,
                rate_hz: t.rate_hz,
                latency_ms: t.latency_ms,
                age_ms: t.history.back().map(|h| (now - h.t_ns) as f64 / 1e6),
            })
            .collect();
        serde_json::json!({
            "updated_unix_ms": SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).map_or(0, |d| d.as_millis() as u64),
            "server": ServerState::from_raw(SERVER_STATE.load(Ordering::SeqCst)),
            "calibration_loaded": s.calibration_ok,
            "trackers": trackers,
        })
    };
    let _ = std::fs::create_dir_all(path.parent().unwrap());
    let tmp = path.with_extension("json.tmp");
    if std::fs::write(&tmp, json.to_string()).is_ok() {
        let _ = std::fs::rename(&tmp, &path);
    }
}

fn reload_calibration(
    shared: &Mutex<Shared>,
    path: Option<&PathBuf>,
    mtime: &mut Option<SystemTime>,
) {
    let Some(path) = path else { return };
    let m = std::fs::metadata(path).and_then(|m| m.modified()).ok();
    if m == *mtime {
        return;
    }
    *mtime = m;
    let (calib, ok) = match m.map(|_| std::fs::read_to_string(path)) {
        None => {
            log(format!(
                "no calibration at {}, using identity",
                path.display()
            ));
            (Calibration::default(), false)
        }
        Some(Ok(text)) => match serde_json::from_str::<Calibration>(&text) {
            Ok(c) => {
                log(format!(
                    "loaded calibration {} (tracker {})",
                    path.display(),
                    c.tracker_serial.as_deref().unwrap_or("?")
                ));
                (c, true)
            }
            Err(e) => {
                log(format!("invalid calibration file: {e}"));
                return;
            }
        },
        Some(Err(e)) => {
            log(format!("cannot read calibration file: {e}"));
            return;
        }
    };
    let mut s = shared.lock().unwrap();
    s.calibration = calib;
    s.calibration_ok = ok;
}

fn register(s: &mut Shared, serial: &str) {
    if !serial.is_empty() && s.known.insert(serial.to_owned()) {
        s.pending.push_back(serial.to_owned());
    }
}

fn normalize(q: [f64; 4]) -> [f64; 4] {
    let n = q.iter().map(|v| v * v).sum::<f64>().sqrt();
    if n < 1e-9 {
        [1.0, 0.0, 0.0, 0.0]
    } else {
        q.map(|v| v / n)
    }
}

/// Quaternion product, (w, x, y, z)
fn qmul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

/// Angular velocity (rad/s, map frame) rotating `old` into `new` over `dt`.
fn angular_velocity(old: [f64; 4], new: [f64; 4], dt: f64) -> [f64; 3] {
    let inv = [old[0], -old[1], -old[2], -old[3]];
    let mut d = qmul(new, inv); // incremental rotation in the map frame
    if d[0] < 0.0 {
        d = d.map(|v| -v);
    }
    let s = (1.0 - d[0] * d[0]).max(0.0).sqrt();
    if s < 1e-9 {
        return [0.0; 3];
    }
    let angle = 2.0 * s.atan2(d[0]);
    [
        d[1] / s * angle / dt,
        d[2] / s * angle / dt,
        d[3] / s * angle / dt,
    ]
}

/// The SDK uses the Isaac Sim frame (X forward, Y left, Z up); OpenVR is X right, Y up, Z back.
/// The change of basis is a proper rotation, so vectors and quaternion vector parts map alike.
fn isaac_to_openvr(v: [f64; 3]) -> [f64; 3] {
    [-v[1], v[2], -v[0]]
}

/// Least-squares slope of positions over time (m/s).
fn fit_velocity(samples: &[&Sample]) -> [f64; 3] {
    let n = samples.len() as f64;
    let t0 = samples.last().unwrap().t_ns;
    let ts: Vec<f64> = samples.iter().map(|s| (s.t_ns - t0) as f64 / 1e9).collect();
    let tm = ts.iter().sum::<f64>() / n;
    let var: f64 = ts.iter().map(|t| (t - tm) * (t - tm)).sum();
    if var < 1e-12 {
        return [0.0; 3];
    }
    let mut v = [0.0; 3];
    for (k, vk) in v.iter_mut().enumerate() {
        let pm = samples.iter().map(|s| s.pos[k]).sum::<f64>() / n;
        *vk = samples
            .iter()
            .zip(&ts)
            .map(|(s, t)| (t - tm) * (s.pos[k] - pm))
            .sum::<f64>()
            / var;
    }
    v
}

fn fill_pose(s: &Shared, serial: &str, out: &mut UtkPose) -> bool {
    let c = &s.calibration;
    let [x, y, z, w] = c.rotation;
    out.world_rot = normalize([w, x, y, z]);
    out.world_pos = c.translation;
    let Some(t) = s.trackers.values().find(|t| t.serial == serial) else {
        return false;
    };
    let running =
        ServerState::from_raw(SERVER_STATE.load(Ordering::SeqCst)) == ServerState::Running;
    out.connected = running && t.present;
    let Some(last) = t.history.back() else {
        out.valid = false;
        return true;
    };
    let age_ns = monotonic_ns() - last.t_ns;
    out.valid = out.connected
        && t.state == STATE_READY
        && t.flags & (FLAG_ROTATION_ONLY | FLAG_SETUP_REQUIRED) == 0
        && age_ns < STALE_NS;
    out.age_s = age_ns as f64 / 1e9;
    out.vel = [0.0; 3];
    out.ang_vel = [0.0; 3];
    if let Some((pos, rot)) = t.held {
        // Completely still: a fixed pose without velocity, so nothing wobbles.
        out.pos = pos;
        out.rot = rot;
        return true;
    }
    out.pos = last.pos;
    out.rot = last.rot;
    let window: Vec<&Sample> = t
        .history
        .iter()
        .filter(|o| last.t_ns - o.t_ns <= VELOCITY_WINDOW_NS)
        .collect();
    if window.len() >= 3 {
        out.vel = fit_velocity(&window);
        let first = window[0];
        let dt = (last.t_ns - first.t_ns) as f64 / 1e9;
        if dt > 1e-3 {
            out.ang_vel = angular_velocity(first.rot, last.rot, dt);
        }
    }
    true
}

fn with_shared<R>(f: impl FnOnce(&mut Shared) -> R) -> Option<R> {
    let d = DRIVER.lock().unwrap();
    let d = d.as_ref()?;
    let mut s = d.shared.lock().unwrap();
    Some(f(&mut s))
}

// ---- C interface used by driver.cpp ----

#[unsafe(no_mangle)]
pub extern "C" fn utk_init(log_fn: LogFn) -> bool {
    let _ = LOG.set(log_fn);
    let mut d = DRIVER.lock().unwrap();
    if d.is_some() {
        return true;
    }
    let shared = Arc::new(Mutex::new(Shared::default()));
    let stop = Arc::new(AtomicBool::new(false));
    let thread = {
        let (shared, stop) = (shared.clone(), stop.clone());
        std::thread::Builder::new()
            .name("utk-housekeeping".into())
            .spawn(move || housekeeping(shared, stop))
    };
    match thread {
        Ok(t) => {
            *d = Some(Driver {
                shared,
                stop,
                thread: Some(t),
            });
            true
        }
        Err(e) => {
            log(format!("cannot start housekeeping thread: {e}"));
            false
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn utk_shutdown() {
    utk_set_server_state(4);
    if let Some(mut d) = DRIVER.lock().unwrap().take() {
        d.stop.store(true, Ordering::SeqCst);
        if let Some(t) = d.thread.take() {
            let _ = t.join();
        }
        write_status(&d.shared);
    }
}

/// 0 starting, 1 running, 2 VIVE Hub running, 3 start failed, 4 stopped.
#[unsafe(no_mangle)]
pub extern "C" fn utk_set_server_state(state: i32) {
    let old = SERVER_STATE.swap(state, Ordering::SeqCst);
    if old != state {
        log(format!(
            "tracker server: {:?}",
            ServerState::from_raw(state)
        ));
    }
}

/// True while VIVE Hub (or its tracker_server) runs: it owns the dongle, so the SDK must not
/// start. vrserver shares the host PID namespace even inside pressure-vessel, so /proc works.
#[unsafe(no_mangle)]
pub extern "C" fn utk_vive_hub_running() -> bool {
    let Ok(dir) = std::fs::read_dir("/proc") else {
        return false;
    };
    dir.flatten().any(|e| {
        e.file_name()
            .to_str()
            .is_some_and(|n| n.bytes().all(|b| b.is_ascii_digit()))
            && std::fs::read_to_string(e.path().join("comm"))
                .is_ok_and(|c| matches!(c.trim_end(), "tracker_server" | "vivehub"))
    })
}

/// # Safety
/// `name` is NUL-terminated or null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn utk_sdk_add(id: u32, name: *const c_char) {
    let serial = if name.is_null() {
        String::new()
    } else {
        unsafe { CStr::from_ptr(name) }
            .to_string_lossy()
            .into_owned()
    };
    log(format!("SDK added tracker {id} {serial}"));
    with_shared(|s| {
        let t = s.trackers.entry(id).or_default();
        t.serial = serial.clone();
        t.present = true;
        register(s, &serial);
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn utk_sdk_remove(id: u32) {
    log(format!("SDK removed tracker {id}"));
    with_shared(|s| {
        if let Some(t) = s.trackers.get_mut(&id) {
            t.present = false;
        }
    });
}

/// Called from the SDK thread for every pose; keep it cheap.
///
/// # Safety
/// `sample` points to a UtkSdkSample.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn utk_sdk_update(sample: *const UtkSdkSample) {
    let d = unsafe { &*sample };
    let now = monotonic_ns();
    let latency_ns = (d.sdk_now_us - d.timestamp_us).clamp(0, 1_000_000) * 1000;
    let pos = isaac_to_openvr(d.location.map(f64::from));
    let [qx, qy, qz, qw] = d.rotation.map(f64::from);
    let [vx, vy, vz] = isaac_to_openvr([qx, qy, qz]);
    let rot = normalize([qw, vx, vy, vz]);
    with_shared(|s| {
        let t = s.trackers.entry(d.id).or_default();
        t.present = true;
        t.state = d.state;
        t.flags = d.flags;
        t.battery = d.battery;
        t.latency_ms = latency_ns as f64 / 1e6;
        t.updates += 1;
        let sample = Sample {
            t_ns: now - latency_ns,
            pos,
            rot,
        };
        if d.flags & FLAG_POSE_STABLE != 0 {
            t.held.get_or_insert((pos, rot));
        } else {
            t.held = None;
        }
        t.history.push_back(sample);
        while t.history.front().is_some_and(|f| now - f.t_ns > HISTORY_NS) {
            t.history.pop_front();
        }
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn utk_sdk_click(id: u32, button: i32, clicks: i32) {
    log(format!("tracker {id} button {button} clicks {clicks}"));
}

/// # Safety
/// `serial` must point to a writable buffer of at least `len` bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn utk_next_new_device(serial: *mut c_char, len: usize) -> bool {
    let Some(Some(next)) = with_shared(|s| s.pending.pop_front()) else {
        return false;
    };
    let bytes = next.as_bytes();
    if bytes.len() + 1 > len {
        return false;
    }
    unsafe {
        std::ptr::copy_nonoverlapping(bytes.as_ptr(), serial as *mut u8, bytes.len());
        *serial.add(bytes.len()) = 0;
    }
    true
}

/// # Safety
/// `serial` must be NUL-terminated and `out` must point to a UtkPose.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn utk_get_pose(serial: *const c_char, out: *mut UtkPose) -> bool {
    let serial = unsafe { CStr::from_ptr(serial) }.to_string_lossy();
    let out = unsafe { &mut *out };
    *out = UtkPose {
        connected: false,
        valid: false,
        age_s: 0.0,
        pos: [0.0; 3],
        rot: [1.0, 0.0, 0.0, 0.0],
        vel: [0.0; 3],
        ang_vel: [0.0; 3],
        world_rot: [1.0, 0.0, 0.0, 0.0],
        world_pos: [0.0; 3],
    };
    with_shared(|s| fill_pose(s, &serial, out)).unwrap_or(false)
}

unsafe extern "C" {
    fn utk_cpp_driver_factory(name: *const c_char, ret: *mut i32) -> *mut std::ffi::c_void;
}

/// Entry point vrserver looks up. Defined here because a cdylib only exports Rust symbols.
///
/// # Safety
/// Called by vrserver with a NUL-terminated interface name.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn HmdDriverFactory(
    name: *const c_char,
    ret: *mut i32,
) -> *mut std::ffi::c_void {
    unsafe { utk_cpp_driver_factory(name, ret) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn angular_velocity_about_y() {
        let half = (0.5f64 * 0.1).sin_cos(); // 0.1 rad
        let q = [half.1, 0.0, half.0, 0.0];
        let w = angular_velocity([1.0, 0.0, 0.0, 0.0], q, 0.1);
        assert!((w[1] - 1.0).abs() < 1e-9 && w[0].abs() < 1e-9 && w[2].abs() < 1e-9);
    }

    #[test]
    fn isaac_up_is_openvr_up() {
        assert_eq!(isaac_to_openvr([0.0, 0.0, 1.0]), [0.0, 1.0, 0.0]); // Z up -> Y up
        assert_eq!(isaac_to_openvr([1.0, 0.0, 0.0]), [-0.0, 0.0, -1.0]); // forward -> -Z
        assert_eq!(isaac_to_openvr([0.0, 1.0, 0.0]), [-1.0, 0.0, -0.0]); // left -> -X
    }

    #[test]
    fn velocity_fit_is_exact_on_a_line() {
        let s: Vec<Sample> = (0..5)
            .map(|i| Sample {
                t_ns: i * 8_000_000,
                pos: [i as f64 * 0.008, 0.0, -(i as f64) * 0.004],
                rot: [1.0, 0.0, 0.0, 0.0],
            })
            .collect();
        let refs: Vec<&Sample> = s.iter().collect();
        let v = fit_velocity(&refs);
        assert!(
            (v[0] - 1.0).abs() < 1e-9 && (v[2] + 0.5).abs() < 1e-9,
            "{v:?}"
        );
    }
}
