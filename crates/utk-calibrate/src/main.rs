//! `utk-calibrate`: in-VR calibration of VIVE Ultimate Trackers against SteamVR.
//!
//! Lives in the SteamVR menu as a dashboard tab (SteamVR starts it with `--dashboard` once
//! `--install` (offline, SteamVR closed) or `--register` (SteamVR running) has been run). Press "Start calibration" there (or A), then hold
//! any controller and any tracker firmly together and draw figure eights: the pair is detected
//! from the motion, data is collected until there is enough rotation, the transform is solved and
//! written to `~/.config/utk-steamvr/calibration.json`, which the SteamVR driver reloads on its
//! own. While calibrating, a panel floats in front of the user; B cancels. Y shows the current
//! calibration.
//!
//! Trackers are read back from SteamVR (the utk driver's devices) and mapped to the tracker map
//! frame with the calibration that was active when the run started; the driver's status file
//! (service state, per-tracker sync state) is shown in the panel.
//!
//! Started by hand it must run in SteamVR's network namespace (`steamvr-netns` when Steam runs in
//! the UU one); started by SteamVR it already does.

mod dashboard;
mod gl;
mod input;
mod log;
mod setup;
mod status;

use std::path::PathBuf;
use std::sync::mpsc;
use std::time::{Duration, Instant};

use anyhow::{Context as _, Result, anyhow};
use ovr_overlay::errors::InitError;
use ovr_overlay::overlay::OverlayHandle;
use ovr_overlay::pose::Matrix3x4;
use ovr_overlay::sys::{
    ETrackedControllerRole, ETrackedDeviceClass, ETrackedDeviceProperty, ETrackingResult,
    ETrackingUniverseOrigin, EVRApplicationType,
};
use ovr_overlay::{Context, TrackedDeviceIndex};
use utk_calib::{Detection, Progress, Recording, Sample, Solution};

pub const APP_KEY: &str = "utk.calibrate";
const PANEL_PX: (u32, u32) = (800, 560);
const ICON_PX: u32 = 256;
const PANEL_WIDTH_M: f32 = 0.55;
/// egui zoom; screen size and pointer positions are given in points (pixels / this).
const PIXELS_PER_POINT: f32 = 1.6;
/// Controller sampling rate; the UI is redrawn every third sample.
const SAMPLE_HZ: f64 = 90.0;
/// Keep this much history for detection and solving.
const KEEP_NS: i64 = 90_000_000_000;
/// Accept a solution only if it is at least this good.
const MAX_RMS_MM: f64 = 20.0;
const MAX_ROT_DEG: f64 = 3.0;
const VREVENT_QUIT: u32 = 700;

#[derive(Debug, Clone)]
enum Phase {
    Idle,
    /// Current calibration file (None if missing or unreadable).
    Showing(Option<serde_json::Value>),
    Waiting,
    Collecting(Detection, Option<Progress>),
    Solving(Detection),
    Saved(Solution),
    Rejected(String, Instant),
}

impl Phase {
    fn calibrating(&self) -> bool {
        matches!(
            self,
            Phase::Waiting | Phase::Collecting(..) | Phase::Solving(_) | Phase::Rejected(..)
        )
    }
}

/// What the user asked for this frame, from controller buttons or panel clicks.
#[derive(Debug, Default, Clone, Copy)]
struct Command {
    start: bool,
    show: bool,
    cancel_or_quit: bool,
}

enum Job {
    Analyze(Recording),
    Solve(Recording, Detection),
}

enum JobResult {
    Analyzed(Vec<Detection>, Option<Progress>),
    Solved(Result<Solution, utk_calib::Error>),
}

fn monotonic_ns() -> i64 {
    let t = nix::time::clock_gettime(nix::time::ClockId::CLOCK_MONOTONIC).expect("CLOCK_MONOTONIC");
    t.tv_sec() * 1_000_000_000 + t.tv_nsec()
}

fn calibration_path() -> PathBuf {
    std::env::var_os("XDG_CONFIG_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))
        .unwrap_or_else(|| PathBuf::from("."))
        .join("utk-steamvr/calibration.json")
}

fn read_calibration() -> Option<serde_json::Value> {
    std::fs::read_to_string(calibration_path())
        .ok()
        .and_then(|t| serde_json::from_str(&t).ok())
}

fn save(sol: &Solution) -> Result<PathBuf> {
    let path = calibration_path();
    std::fs::create_dir_all(path.parent().unwrap())?;
    if path.exists() {
        std::fs::copy(&path, path.with_extension("json.prev"))?;
    }
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)?
        .as_secs();
    let tmp = path.with_extension("json.tmp");
    std::fs::write(
        &tmp,
        serde_json::to_string_pretty(&sol.to_json(now))? + "\n",
    )?;
    std::fs::rename(&tmp, &path)?;
    Ok(path)
}

/// Accepts a detection only when it clearly beats every other pair sharing the controller or tracker.
fn accept(dets: &[Detection]) -> Option<Detection> {
    let best = dets.first()?;
    if best.correlation < 0.9 {
        return None;
    }
    let runner_up = dets[1..]
        .iter()
        .filter(|d| d.controller == best.controller || d.tracker == best.tracker)
        .map(|d| d.correlation)
        .fold(f64::NEG_INFINITY, f64::max);
    (best.correlation - runner_up >= 0.3).then(|| best.clone())
}

fn worker(jobs: mpsc::Receiver<Job>, results: mpsc::Sender<JobResult>) {
    while let Ok(mut job) = jobs.recv() {
        // Only the newest analysis request matters.
        while let Ok(next) = jobs.try_recv() {
            if matches!(job, Job::Analyze(_)) {
                job = next;
            }
        }
        let out = match job {
            Job::Analyze(rec) => {
                let dets = utk_calib::detect(&rec);
                let prog = accept(&dets).map(|d| utk_calib::progress(&rec, &d));
                JobResult::Analyzed(dets, prog)
            }
            Job::Solve(rec, det) => JobResult::Solved(utk_calib::solve(&rec, &det)),
        };
        if results.send(out).is_err() {
            return;
        }
    }
}

/// Rotation part of a row-major 3x4 matrix as (x, y, z, w).
fn matrix_to_quat(m: &[[f32; 4]; 3]) -> [f64; 4] {
    let m = |r: usize, c: usize| f64::from(m[r][c]);
    let trace = m(0, 0) + m(1, 1) + m(2, 2);
    let q = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        [
            (m(2, 1) - m(1, 2)) / s,
            (m(0, 2) - m(2, 0)) / s,
            (m(1, 0) - m(0, 1)) / s,
            0.25 * s,
        ]
    } else if m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2) {
        let s = (1.0 + m(0, 0) - m(1, 1) - m(2, 2)).sqrt() * 2.0;
        [
            0.25 * s,
            (m(0, 1) + m(1, 0)) / s,
            (m(0, 2) + m(2, 0)) / s,
            (m(2, 1) - m(1, 2)) / s,
        ]
    } else if m(1, 1) > m(2, 2) {
        let s = (1.0 + m(1, 1) - m(0, 0) - m(2, 2)).sqrt() * 2.0;
        [
            (m(0, 1) + m(1, 0)) / s,
            0.25 * s,
            (m(1, 2) + m(2, 1)) / s,
            (m(0, 2) - m(2, 0)) / s,
        ]
    } else {
        let s = (1.0 + m(2, 2) - m(0, 0) - m(1, 1)).sqrt() * 2.0;
        [
            (m(0, 2) + m(2, 0)) / s,
            (m(1, 2) + m(2, 1)) / s,
            0.25 * s,
            (m(1, 0) - m(0, 1)) / s,
        ]
    };
    let n = q.iter().map(|v| v * v).sum::<f64>().sqrt();
    q.map(|v| v / n)
}

/// Panel pose: `dist` meters in front of the head (horizontal), slightly below eye level, facing it.
fn panel_transform(hmd: &[[f32; 4]; 3], dist: f32) -> Matrix3x4 {
    let pos = [hmd[0][3], hmd[1][3], hmd[2][3]];
    // HMD looks along -Z; flatten to the horizontal plane.
    let (fx, fz) = (-hmd[0][2], -hmd[2][2]);
    let n = (fx * fx + fz * fz).sqrt().max(1e-3);
    let (fx, fz) = (fx / n, fz / n);
    let center = [pos[0] + fx * dist, pos[1] - 0.15, pos[2] + fz * dist];
    // Overlay faces +Z; make its +Z point back to the user: z = -forward, x = up × z.
    let z = [-fx, 0.0, -fz];
    let x = [z[2], 0.0, -z[0]];
    Matrix3x4([
        [x[0], 0.0, z[0], center[0]],
        [x[1], 1.0, z[1], center[1]],
        [x[2], 0.0, z[2], center[2]],
    ])
}

/// Angle between the head's horizontal forward direction and the direction to the panel.
fn off_axis_deg(hmd: &[[f32; 4]; 3], panel: &Matrix3x4) -> f32 {
    let (fx, fz) = (-hmd[0][2], -hmd[2][2]);
    let (dx, dz) = (panel.0[0][3] - hmd[0][3], panel.0[2][3] - hmd[2][3]);
    let dot =
        (fx * dx + fz * dz) / ((fx * fx + fz * fz).sqrt() * (dx * dx + dz * dz).sqrt()).max(1e-6);
    dot.clamp(-1.0, 1.0).acos().to_degrees()
}

/// Calibration as a transform: p_steamvr = R(rot) · p_map + trans, rot as (x, y, z, w).
#[derive(Debug, Clone, Copy)]
struct Xform {
    rot: [f64; 4],
    trans: [f64; 3],
}

impl Xform {
    const IDENTITY: Self = Self {
        rot: [0.0, 0.0, 0.0, 1.0],
        trans: [0.0; 3],
    };

    fn from_calibration(c: Option<&serde_json::Value>) -> Self {
        let get = |k: &str, n: usize| -> Option<Vec<f64>> {
            let v: Vec<f64> = c?
                .get(k)?
                .as_array()?
                .iter()
                .filter_map(|x| x.as_f64())
                .collect();
            (v.len() == n).then_some(v)
        };
        match (get("rotation", 4), get("translation", 3)) {
            (Some(r), Some(t)) => Self {
                rot: [r[0], r[1], r[2], r[3]],
                trans: [t[0], t[1], t[2]],
            },
            _ => Self::IDENTITY,
        }
    }

    /// Maps a pose reported by SteamVR back to the tracker map frame: X⁻¹ · pose.
    fn to_map(self, pos: [f64; 3], rot: [f64; 4]) -> ([f64; 3], [f64; 4]) {
        let inv = [-self.rot[0], -self.rot[1], -self.rot[2], self.rot[3]];
        let d = [
            pos[0] - self.trans[0],
            pos[1] - self.trans[1],
            pos[2] - self.trans[2],
        ];
        (rotate(inv, d), qmul_xyzw(inv, rot))
    }
}

/// Quaternion product, (x, y, z, w).
fn qmul_xyzw(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let [ax, ay, az, aw] = a;
    let [bx, by, bz, bw] = b;
    [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]
}

fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let r = qmul_xyzw(
        qmul_xyzw(q, [v[0], v[1], v[2], 0.0]),
        [-q[0], -q[1], -q[2], q[3]],
    );
    [r[0], r[1], r[2]]
}

/// A SteamVR device we sample: a controller, or a tracker of the utk driver.
struct Device {
    serial: String,
    /// "left", "right", "other" for controllers; None for trackers.
    role: Option<&'static str>,
}

fn main() -> Result<()> {
    log::init();
    let args: Vec<String> = std::env::args().skip(1).collect();
    log!("utk-calibrate {} {:?}", env!("CARGO_PKG_VERSION"), args);
    let register = args.iter().any(|a| a == "--register");
    // Started by SteamVR from the dashboard manifest: stay resident, B never quits.
    let resident = args.iter().any(|a| a == "--dashboard");

    if args.iter().any(|a| a == "--install") {
        return setup::install();
    }
    if args.iter().any(|a| a == "--uninstall") {
        return setup::uninstall();
    }
    if let Some(i) = args.iter().position(|a| a == "--snapshot") {
        let path = args
            .get(i + 1)
            .ok_or_else(|| anyhow!("--snapshot needs an output path"))?;
        return snapshot_png(std::path::Path::new(path));
    }

    let app_type = if register {
        EVRApplicationType::VRApplication_Utility
    } else {
        EVRApplicationType::VRApplication_Overlay
    };
    let ctx = Context::init(app_type).map_err(|e| match e {
        InitError::Sys(e) => anyhow!(
            "cannot connect to SteamVR (is it running, and in this network namespace?): {e}"
        ),
        InitError::AlreadyInitialized => anyhow!("OpenVR is already initialized"),
    })?;

    if register {
        let exe = std::env::current_exe()?.canonicalize()?;
        let actions = input::write_manifest()?;
        let icon = gl::Gl::new().and_then(|gl| write_icon(&gl)).ok();
        let manifest = dashboard::register(
            &mut ctx.applications_mngr(),
            &exe,
            &actions,
            icon.as_deref(),
        )?;
        println!(
            "registered {} ({APP_KEY}); SteamVR starts it with the menu from now on",
            manifest.display()
        );
        return Ok(());
    }

    let mut overlays = ctx.overlay_mngr();
    let mut system = ctx.system_mngr();
    let mut input_mngr = ctx.input_mngr();
    let buttons = input::Buttons::new(&mut input_mngr)?;

    let gl = gl::Gl::new().context("creating the OpenGL context")?;
    log!("OpenGL: {}", unsafe {
        egui_glow::glow::HasContext::get_parameter_string(&*gl.glow, egui_glow::glow::RENDERER)
    });
    let target = gl::Target::new(&gl.glow, PANEL_PX.0, PANEL_PX.1)?;
    let mut painter = egui_glow::Painter::new(gl.glow.clone(), "", None, false)
        .map_err(|e| anyhow!("egui painter: {e}"))?;
    let egui = egui::Context::default();
    egui.set_visuals(egui::Visuals::dark());
    egui.set_pixels_per_point(PIXELS_PER_POINT);

    // No texture-bounds flip: SteamVR already shows OpenGL overlay textures the right way up,
    // and its mouse coordinates are in the same (unflipped) texture space.

    // Tab in the SteamVR menu.
    let dash = dashboard::Dashboard::new(APP_KEY, "UTK Calibration", PANEL_PX, PIXELS_PER_POINT)?;
    // Floating panel shown while calibrating with the menu closed.
    // ovr_overlay passes these strings straight to C, so they must be NUL-terminated.
    let world = overlays
        .create_overlay("utk.calibrate.panel\0", "UTK calibration\0")
        .map_err(|e| anyhow!("creating the overlay: {e}"))?;
    overlays.set_width(world, PANEL_WIDTH_M).ok();

    // Menu-bar icon. A texture submitted once is not kept for dashboard thumbnails (SteamVR showed
    // a question mark), so render it to a PNG and let SteamVR load the file.
    match write_icon(&gl) {
        Ok(path) => {
            if let Err(e) = dash.set_thumbnail_file(&path) {
                log!("thumbnail: {e}");
            }
        }
        Err(e) => log!("icon: {e}"),
    }

    let (job_tx, job_rx) = mpsc::channel();
    let (res_tx, res_rx) = mpsc::channel();
    std::thread::spawn(move || worker(job_rx, res_tx));

    let mut devices: Vec<Option<Device>> = (0..ovr_overlay::sys::k_unMaxTrackedDeviceCount)
        .map(|_| None)
        .collect();
    let mut rec = Recording::default();
    let mut phase = Phase::Idle;
    let mut current = read_calibration();
    // Calibration active when the current run started; tracker poses are mapped back with it.
    let mut run_xform = Xform::from_calibration(current.as_ref());
    let mut status = status::read();
    let mut last_status = Instant::now();
    let mut last_dets: Vec<Detection> = Vec::new();
    let mut panel: Option<Matrix3x4> = None;
    let mut world_visible = false;
    let mut off_since: Option<Instant> = None;
    let mut last_analysis = Instant::now();
    let mut analysis_pending = false;
    let mut pointer: Vec<egui::Event> = Vec::new();
    let mut clicked = Command::default();
    let period = Duration::from_secs_f64(1.0 / SAMPLE_HZ);
    let mut next = Instant::now();
    let mut tick: u64 = 0;
    let mut last_active = [false; 3];
    let mut last_dashboard_open = false;

    loop {
        while let Some(ev) = system.poll_next_event() {
            if ev.event_type == VREVENT_QUIT {
                return Ok(());
            }
        }

        // Sample controllers and the utk trackers in SteamVR's raw universe at the same instant.
        let now_ns = monotonic_ns();
        let raw = system.get_device_to_absolute_tracking_pose(
            ETrackingUniverseOrigin::TrackingUniverseRawAndUncalibrated,
            0.0,
        );
        for (i, pose) in raw.iter().enumerate() {
            if !pose.bDeviceIsConnected {
                devices[i] = None;
                continue;
            }
            let index = TrackedDeviceIndex(i as u32);
            if devices[i].is_none() {
                let class = system.get_tracked_device_class(index);
                let serial = system
                    .get_tracked_device_property::<String>(
                        index,
                        ETrackedDeviceProperty::Prop_SerialNumber_String,
                    )
                    .unwrap_or_default();
                devices[i] = match class {
                    ETrackedDeviceClass::TrackedDeviceClass_Controller => Some(Device {
                        serial,
                        role: Some(
                            match system.get_controller_role_for_tracked_device_index(index) {
                                ETrackedControllerRole::TrackedControllerRole_LeftHand => "left",
                                ETrackedControllerRole::TrackedControllerRole_RightHand => "right",
                                _ => "other",
                            },
                        ),
                    }),
                    ETrackedDeviceClass::TrackedDeviceClass_GenericTracker
                        if system
                            .get_tracked_device_property::<String>(
                                index,
                                ETrackedDeviceProperty::Prop_TrackingSystemName_String,
                            )
                            .is_ok_and(|n| n == "utk") =>
                    {
                        Some(Device { serial, role: None })
                    }
                    _ => continue,
                };
            }
            let Some(d) = &devices[i] else { continue };
            if !pose.bPoseIsValid
                || pose.eTrackingResult != ETrackingResult::TrackingResult_Running_OK
            {
                continue;
            }
            let m = pose.mDeviceToAbsoluteTracking.m;
            let (pos, rot) = (
                [m[0][3], m[1][3], m[2][3]].map(f64::from),
                matrix_to_quat(&m),
            );
            let sample = if d.role.is_some() {
                Sample {
                    t_ns: now_ns,
                    pos,
                    rot,
                }
            } else {
                let (pos, rot) = run_xform.to_map(pos, rot);
                Sample {
                    t_ns: now_ns,
                    pos,
                    rot,
                }
            };
            let map = if d.role.is_some() {
                &mut rec.controllers
            } else {
                &mut rec.trackers
            };
            map.entry(d.serial.clone()).or_default().push(sample);
        }
        for v in rec
            .controllers
            .values_mut()
            .chain(rec.trackers.values_mut())
        {
            let cut = v.partition_point(|s| s.t_ns < now_ns - KEEP_NS);
            v.drain(..cut);
        }
        if last_status.elapsed() > Duration::from_millis(500) {
            status = status::read();
            last_status = Instant::now();
        }

        // Commands: controller buttons (A start, Y show, B cancel/quit) or clicks in the menu tab.
        let pressed = buttons.poll(&mut input_mngr);
        if pressed.active != last_active {
            log!("input active (start, quit, show) = {:?}", pressed.active);
            last_active = pressed.active;
        }
        if pressed.start || pressed.quit || pressed.show {
            log!(
                "button: start={} quit={} show={}",
                pressed.start,
                pressed.quit,
                pressed.show
            );
        }
        let cmd = Command {
            start: pressed.start || clicked.start,
            show: pressed.show || clicked.show,
            cancel_or_quit: pressed.quit || clicked.cancel_or_quit,
        };
        clicked = Command::default();
        if cmd.cancel_or_quit {
            if phase.calibrating() || matches!(phase, Phase::Showing(_) | Phase::Saved(_)) {
                phase = Phase::Idle;
            } else if !resident {
                return Ok(());
            }
        } else if cmd.start && !phase.calibrating() {
            rec = Recording::default();
            current = read_calibration();
            run_xform = Xform::from_calibration(current.as_ref());
            last_dets.clear();
            phase = Phase::Waiting;
        } else if cmd.show && !phase.calibrating() {
            current = read_calibration();
            phase = match phase {
                Phase::Showing(_) => Phase::Idle,
                _ => Phase::Showing(current.clone()),
            };
        }

        // Results from the worker.
        while let Ok(r) = res_rx.try_recv() {
            match r {
                JobResult::Analyzed(dets, prog) => {
                    analysis_pending = false;
                    let accepted = accept(&dets);
                    last_dets = dets;
                    match (&phase, accepted) {
                        (Phase::Waiting | Phase::Collecting(..), Some(det)) => {
                            if prog.as_ref().is_some_and(|p| p.ready) {
                                job_tx.send(Job::Solve(rec.clone(), det.clone())).ok();
                                phase = Phase::Solving(det);
                            } else {
                                phase = Phase::Collecting(det, prog);
                            }
                        }
                        (Phase::Collecting(..), None) => phase = Phase::Waiting,
                        _ => {}
                    }
                }
                JobResult::Solved(_) if !matches!(phase, Phase::Solving(_)) => {}
                JobResult::Solved(Ok(sol))
                    if sol.rms_mm <= MAX_RMS_MM && sol.rot_median_deg <= MAX_ROT_DEG =>
                {
                    phase = match save(&sol) {
                        Ok(_) => {
                            current = read_calibration();
                            Phase::Saved(sol)
                        }
                        Err(e) => Phase::Rejected(
                            format!("Could not save the calibration: {e}"),
                            Instant::now(),
                        ),
                    };
                }
                JobResult::Solved(Ok(sol)) => {
                    phase = Phase::Rejected(
                        format!(
                            "Not accurate enough ({:.1} mm, {:.1}°). Hold the pair tighter and draw again.",
                            sol.rms_mm, sol.rot_median_deg
                        ),
                        Instant::now(),
                    );
                    rec = Recording::default();
                }
                JobResult::Solved(Err(e)) => {
                    phase = Phase::Rejected(
                        format!("Calibration failed: {e}. Draw again."),
                        Instant::now(),
                    );
                    rec = Recording::default();
                }
            }
        }
        if let Phase::Rejected(_, at) = &phase
            && at.elapsed() > Duration::from_secs(4)
        {
            phase = Phase::Waiting;
        }
        if !analysis_pending
            && matches!(phase, Phase::Waiting | Phase::Collecting(..))
            && last_analysis.elapsed() > Duration::from_millis(500)
        {
            job_tx.send(Job::Analyze(rec.clone())).ok();
            analysis_pending = true;
            last_analysis = Instant::now();
        }

        pointer.extend(dash.poll());

        if tick.is_multiple_of(3) {
            let dashboard_open = overlays.is_dashboard_visible();
            if dashboard_open != last_dashboard_open {
                log!(
                    "SteamVR menu {}",
                    if dashboard_open { "opened" } else { "closed" }
                );
                last_dashboard_open = dashboard_open;
            }
            // The floating panel is for calibrating (and its result) with the menu closed.
            let want_world =
                (phase.calibrating() || matches!(phase, Phase::Saved(_))) && !dashboard_open;
            if want_world {
                let standing = system.get_device_to_absolute_tracking_pose(
                    ETrackingUniverseOrigin::TrackingUniverseStanding,
                    0.0,
                );
                if standing[0].bPoseIsValid {
                    let m = standing[0].mDeviceToAbsoluteTracking.m;
                    // Place the panel when it appears, and again after looking away for a second.
                    let off = panel.as_ref().map_or(f32::MAX, |p| off_axis_deg(&m, p));
                    if !world_visible
                        || off > 50.0
                            && off_since.is_some_and(|t| t.elapsed() > Duration::from_secs(1))
                    {
                        let t = panel_transform(&m, 1.0);
                        overlays
                            .set_transform_absolute(
                                world,
                                ETrackingUniverseOrigin::TrackingUniverseStanding,
                                &t,
                            )
                            .ok();
                        panel = Some(t);
                        off_since = None;
                    } else if off > 50.0 && off_since.is_none() {
                        off_since = Some(Instant::now());
                    } else if off <= 50.0 {
                        off_since = None;
                    }
                }
            }
            if want_world != world_visible {
                overlays.set_visibility(world, want_world).ok();
                world_visible = want_world;
            }

            if dash.is_active() || world_visible {
                let view = View {
                    phase: &phase,
                    current: current.as_ref(),
                    dets: &last_dets,
                    devices: &devices,
                    status: status.as_ref(),
                    resident,
                    in_menu: !world_visible,
                };
                let input = egui::RawInput {
                    screen_rect: Some(egui::Rect::from_min_size(
                        egui::Pos2::ZERO,
                        egui::vec2(PANEL_PX.0 as f32, PANEL_PX.1 as f32) / PIXELS_PER_POINT,
                    )),
                    events: std::mem::take(&mut pointer),
                    ..Default::default()
                };
                let mut out = egui.run_ui(input, |ui| clicked = view.ui(ui));
                let prims = egui.tessellate(out.shapes, out.pixels_per_point);
                target.bind(&gl.glow);
                painter.paint_and_update_textures(
                    [PANEL_PX.0, PANEL_PX.1],
                    out.pixels_per_point,
                    &prims,
                    &mut out.textures_delta,
                );
                target.finish(&gl.glow);
                overlays
                    .set_image_opengl(OverlayHandle(dash.main), target.gl_name())
                    .ok();
                if world_visible {
                    overlays.set_image_opengl(world, target.gl_name()).ok();
                }
            } else {
                pointer.clear();
            }
        }

        tick += 1;
        next += period;
        let now = Instant::now();
        if next > now {
            std::thread::sleep(next - now);
        } else {
            next = now;
        }
    }
}

/// Debug aid: renders the idle panel without SteamVR and writes it as a PNG.
fn snapshot_png(path: &std::path::Path) -> Result<()> {
    let gl = gl::Gl::new().context("creating the OpenGL context")?;
    let target = gl::Target::new(&gl.glow, PANEL_PX.0, PANEL_PX.1)?;
    let mut painter = egui_glow::Painter::new(gl.glow.clone(), "", None, false)
        .map_err(|e| anyhow!("egui painter: {e}"))?;
    let egui = egui::Context::default();
    egui.set_visuals(egui::Visuals::dark());
    egui.set_pixels_per_point(PIXELS_PER_POINT);
    let current = read_calibration();
    let driver_status = status::read();
    let view = View {
        phase: &Phase::Idle,
        current: current.as_ref(),
        dets: &[],
        devices: &[],
        status: driver_status.as_ref(),
        resident: true,
        in_menu: true,
    };
    // Two frames: egui lays text out on the first one.
    for _ in 0..2 {
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(PANEL_PX.0 as f32, PANEL_PX.1 as f32) / PIXELS_PER_POINT,
            )),
            ..Default::default()
        };
        let mut out = egui.run_ui(input, |ui| {
            view.ui(ui);
        });
        let prims = egui.tessellate(out.shapes, out.pixels_per_point);
        target.bind(&gl.glow);
        painter.paint_and_update_textures(
            [PANEL_PX.0, PANEL_PX.1],
            out.pixels_per_point,
            &prims,
            &mut out.textures_delta,
        );
    }
    let pixels = target.read_rgba(&gl.glow);
    target.finish(&gl.glow);
    write_png(path, PANEL_PX.0, PANEL_PX.1, &pixels)?;
    if let Some(icon) = path.to_str().map(|p| p.replace(".png", "-icon.png")) {
        render_png(&gl, ICON_PX, std::path::Path::new(&icon), draw_icon)?;
    }
    println!("wrote {}", path.display());
    Ok(())
}

fn data_dir() -> PathBuf {
    std::env::var_os("XDG_DATA_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".local/share")))
        .unwrap_or_else(|| PathBuf::from("."))
        .join("utk-steamvr")
}

/// Renders `draw` into a `size`×`size` RGBA PNG with its own egui context and painter.
fn render_png(
    gl: &gl::Gl,
    size: u32,
    path: &std::path::Path,
    draw: impl FnOnce(&egui::Painter, egui::Rect),
) -> Result<()> {
    let target = gl::Target::new(&gl.glow, size, size)?;
    // Own painter: egui contexts all number their font atlas TextureId::Managed(0), so sharing a
    // painter with the panel would overwrite its font texture.
    let mut painter = egui_glow::Painter::new(gl.glow.clone(), "", None, false)
        .map_err(|e| anyhow!("egui painter: {e}"))?;
    let ctx = egui::Context::default();
    let rect = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(size as f32, size as f32));
    let mut draw = Some(draw);
    let mut out = ctx.run_ui(
        egui::RawInput {
            screen_rect: Some(rect),
            ..Default::default()
        },
        |ui| {
            if let Some(d) = draw.take() {
                d(ui.painter(), rect);
            }
        },
    );
    let prims = ctx.tessellate(out.shapes, out.pixels_per_point);
    target.bind(&gl.glow);
    painter.paint_and_update_textures(
        [size, size],
        out.pixels_per_point,
        &prims,
        &mut out.textures_delta,
    );
    let pixels = target.read_rgba(&gl.glow);
    target.finish(&gl.glow);
    painter.destroy();
    write_png(path, size, size, &pixels)
}

/// Writes GL read-back pixels (bottom row first) as a PNG.
fn write_png(path: &std::path::Path, width: u32, height: u32, pixels: &[u8]) -> Result<()> {
    let row = width as usize * 4;
    let flipped: Vec<u8> = pixels.chunks(row).rev().flatten().copied().collect();
    let tmp = path.with_extension("png.tmp");
    let mut enc = png::Encoder::new(
        std::io::BufWriter::new(std::fs::File::create(&tmp)?),
        width,
        height,
    );
    enc.set_color(png::ColorType::Rgba);
    enc.set_depth(png::BitDepth::Eight);
    enc.write_header()?.write_image_data(&flipped)?;
    std::fs::rename(&tmp, path)?;
    Ok(())
}

/// Writes the app icon (a stylised tracker) to ~/.local/share/utk-steamvr/icon.png.
fn write_icon(gl: &gl::Gl) -> Result<PathBuf> {
    let path = data_dir().join("icon.png");
    std::fs::create_dir_all(path.parent().unwrap())?;
    render_png(gl, ICON_PX, &path, draw_icon)?;
    Ok(path)
}

fn draw_icon(p: &egui::Painter, rect: egui::Rect) {
    let s = rect.width();
    let at = |x: f32, y: f32| egui::pos2(rect.min.x + x * s, rect.min.y + y * s);
    let blue = egui::Color32::from_rgb(90, 170, 255);
    p.circle_filled(at(0.5, 0.5), 0.47 * s, egui::Color32::from_rgb(28, 36, 48));
    p.circle_stroke(at(0.5, 0.5), 0.47 * s, egui::Stroke::new(0.025 * s, blue));
    // Tracker body with its camera.
    p.circle_filled(
        at(0.5, 0.42),
        0.22 * s,
        egui::Color32::from_rgb(235, 238, 242),
    );
    p.circle_filled(
        at(0.5, 0.42),
        0.085 * s,
        egui::Color32::from_rgb(28, 36, 48),
    );
    p.circle_filled(at(0.5, 0.42), 0.035 * s, blue);
    p.text(
        at(0.5, 0.79),
        egui::Align2::CENTER_CENTER,
        "UTK",
        egui::FontId::proportional(0.17 * s),
        egui::Color32::WHITE,
    );
}

fn human_duration(secs: u64) -> String {
    match secs {
        s if s < 120 => format!("{s} s"),
        s if s < 7200 => format!("{} min", s / 60),
        s if s < 172_800 => format!("{} h", s / 3600),
        s => format!("{} days", s / 86_400),
    }
}

struct View<'a> {
    phase: &'a Phase,
    current: Option<&'a serde_json::Value>,
    dets: &'a [Detection],
    devices: &'a [Option<Device>],
    status: Option<&'a status::Status>,
    resident: bool,
    /// Shown in the SteamVR menu: controller buttons do not reach us there, so no key hints.
    in_menu: bool,
}

impl View<'_> {
    fn controller_name(&self, serial: &str) -> String {
        self.devices
            .iter()
            .flatten()
            .find(|d| d.serial == serial)
            .and_then(|d| d.role)
            .map_or_else(|| serial.to_owned(), |r| format!("{r} controller"))
    }

    fn calibration_summary(&self, ui: &mut egui::Ui, cal: Option<&serde_json::Value>) {
        let Some(c) = cal else {
            ui.label("No calibration yet: trackers use the identity transform.");
            return;
        };
        let f = |k: &str| c.get(k).cloned().unwrap_or(serde_json::Value::Null);
        let s = |k: &str| f(k).as_str().map_or_else(|| "?".to_owned(), str::to_owned);
        ui.label(format!(
            "Tracker {} with {}",
            s("tracker_serial"),
            self.controller_name(&s("controller_serial"))
        ));
        ui.label(format!(
            "Error {:.1} mm, {:.1}°",
            f("rms_mm").as_f64().unwrap_or(f64::NAN),
            f("rot_median_deg").as_f64().unwrap_or(f64::NAN)
        ));
        if let Some(t) = f("created_unix").as_u64() {
            let now = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(0, |d| d.as_secs());
            ui.label(format!(
                "Created {} ago",
                human_duration(now.saturating_sub(t))
            ));
        }
    }

    fn devices_ui(&self, ui: &mut egui::Ui) {
        let red = egui::Color32::from_rgb(255, 120, 120);
        let amber = egui::Color32::from_rgb(255, 190, 90);
        let status = self.status.filter(|s| s.is_fresh());
        match status.map(|s| s.server.as_str()) {
            None => {
                ui.colored_label(red, "The utk SteamVR driver is not running. Enable it under SteamVR Settings > Startup / Shutdown > Manage Add-ons.");
            }
            Some("vive_hub_running") => {
                ui.colored_label(red, "VIVE Hub is running and holds the tracker dongle. Close VIVE Hub; the trackers start automatically.");
            }
            Some("start_failed") => {
                ui.colored_label(
                    red,
                    "Could not start the tracker service. Is the dongle plugged in?",
                );
            }
            Some("starting") => {
                ui.colored_label(amber, "Starting the tracker service…");
            }
            _ => {}
        }
        egui::Grid::new("trackers")
            .num_columns(5)
            .spacing([14.0, 2.0])
            .show(ui, |ui| {
                for t in status.map_or(&[][..], |s| &s.trackers[..]) {
                    ui.label(format!(
                        "{}{}",
                        t.serial,
                        if t.host { " (host)" } else { "" }
                    ));
                    if !t.present {
                        ui.colored_label(amber, "Not connected");
                    } else if t.state == "ready" && !t.rotation_only {
                        ui.colored_label(egui::Color32::LIGHT_GREEN, t.state_label());
                    } else {
                        ui.colored_label(amber, t.state_label());
                    }
                    ui.label(format!("{}%", t.battery));
                    ui.weak(format!("{:.0} Hz", t.rate_hz));
                    ui.weak(format!("{:.0} ms", t.latency_ms));
                    ui.end_row();
                }
                for d in self.devices.iter().flatten() {
                    if let Some(role) = d.role {
                        ui.label(format!("{role} controller"));
                        ui.weak(&d.serial);
                        ui.end_row();
                    }
                }
            });
    }

    /// Draws the panel and returns the buttons clicked with the laser pointer.
    fn ui(&self, root: &mut egui::Ui) -> Command {
        let mut cmd = Command::default();
        egui::CentralPanel::default().show(root, |ui| {
            ui.heading("UTK calibration");
            ui.add_space(6.0);
            match self.phase {
                Phase::Idle => {
                    ui.label("Calibrate the trackers against your headset's tracking.");
                    ui.label("You will hold one controller and one tracker together and draw figure eights.");
                }
                Phase::Showing(cal) => {
                    ui.label("Current calibration");
                    self.calibration_summary(ui, cal.as_ref());
                }
                Phase::Waiting => {
                    ui.label("Hold one controller and one tracker firmly together.");
                    ui.label("Draw large figure eights and twist your wrist in every direction.");
                    ui.weak("The pair is detected from the motion.");
                }
                Phase::Collecting(det, prog) => {
                    ui.label(format!("Tracker {} is moving with the {}.", det.tracker, self.controller_name(&det.controller)));
                    ui.label("Keep drawing figure eights and twisting your wrist.");
                    if let Some(p) = prog {
                        ui.add_space(4.0);
                        ui.add(egui::ProgressBar::new((p.held_seconds / utk_calib::READY_HELD_S).min(1.0) as f32).text(format!("Motion {:.0} s", p.held_seconds)));
                        ui.add(
                            egui::ProgressBar::new((p.rotation_diversity / utk_calib::READY_DIVERSITY).min(1.0) as f32)
                                .text(format!("Rotation coverage {:.0}%", p.rotation_diversity * 100.0)),
                        );
                    }
                }
                Phase::Solving(det) => {
                    ui.horizontal(|ui| {
                        ui.spinner();
                        ui.label(format!("Solving with tracker {}…", det.tracker));
                    });
                }
                Phase::Saved(sol) => {
                    ui.colored_label(egui::Color32::LIGHT_GREEN, "Calibration saved.");
                    ui.label(format!(
                        "Error {:.1} mm, {:.1}° (tracker {}, {}).",
                        sol.rms_mm,
                        sol.rot_median_deg,
                        sol.tracker,
                        self.controller_name(&sol.controller)
                    ));
                    ui.label("The SteamVR driver picks it up within a second.");
                }
                Phase::Rejected(msg, _) => {
                    ui.colored_label(egui::Color32::from_rgb(255, 170, 80), msg);
                }
            }

            ui.add_space(8.0);
            if self.in_menu {
                ui.horizontal(|ui| {
                    if self.phase.calibrating() {
                        if ui.button("Cancel").clicked() {
                            cmd.cancel_or_quit = true;
                        }
                    } else {
                        if ui.button("Start calibration").clicked() {
                            cmd.start = true;
                        }
                        let show = if matches!(self.phase, Phase::Showing(_)) { "Back" } else { "Current calibration" };
                        if ui.button(show).clicked() {
                            cmd.show = true;
                        }
                        if !self.resident && ui.button("Quit").clicked() {
                            cmd.cancel_or_quit = true;
                        }
                    }
                });
                ui.weak("With the menu closed: A start, Y current calibration, B cancel.");
            } else {
                ui.strong(if self.phase.calibrating() {
                    "B  Cancel"
                } else {
                    "A  Calibrate again      Y  Current calibration      B  Close"
                });
            }

            ui.add_space(8.0);
            ui.separator();
            self.devices_ui(ui);
            if self.phase.calibrating() {
                if let Some(best) = self.dets.first() {
                    ui.weak(format!("Best match: {} + {} ({:.2})", best.tracker, self.controller_name(&best.controller), best.correlation));
                }
            } else if !matches!(self.phase, Phase::Showing(_)) {
                ui.add_space(4.0);
                ui.weak("Current calibration:");
                self.calibration_summary(ui, self.current);
            }
        });
        cmd
    }
}
