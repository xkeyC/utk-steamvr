//! Calibration of the VIVE Ultimate Tracker map frame against SteamVR's raw tracking frame.
//!
//! The user holds a SteamVR controller and a tracker together and draws figure eights. Model:
//! `C(t) = X · T(t + lag) · Y`, where `C` is the controller pose in SteamVR raw space, `T` the
//! tracker pose in its map frame, `X` the map → SteamVR transform we want and `Y` the fixed
//! tracker → controller offset.
//!
//! API contract (implemented in this crate, used by `utk-calibrate`):
//! - [`retime_batches`]: tracker_server sends poses in ~50 ms batches whose timestamps are nearly
//!   identical; spread each batch over the previous interval (same rule as the SteamVR driver).
//! - [`detect`]: find which controller and tracker are held together, without any button press.
//! - [`progress`]: how much usable data has been collected, for a progress bar.
//! - [`solve`]: compute `X` (yaw-only rotation + translation), the lag and residuals.

use std::collections::BTreeMap;

mod detect;
mod math;
mod solver;
mod track;

use track::Track;

/// Quaternion (x, y, z, w), the order used in calibration.json.
pub type Quat = [f64; 4];

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    /// CLOCK_MONOTONIC, ns.
    pub t_ns: i64,
    /// Meters.
    pub pos: [f64; 3],
    pub rot: Quat,
}

/// Collected poses. Controller samples come from SteamVR (raw universe, prediction 0, `t_ns` =
/// sampling time); tracker samples come from tracker_server and must already be re-timed with
/// [`retime_batches`]. Keys are serial numbers.
#[derive(Debug, Clone, Default)]
pub struct Recording {
    pub controllers: BTreeMap<String, Vec<Sample>>,
    pub trackers: BTreeMap<String, Vec<Sample>>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Detection {
    pub controller: String,
    pub tracker: String,
    /// Correlation of the angular-speed series (1 = moving together).
    pub correlation: f64,
    /// Tracker time offset (see [`Solution::lag_ms`]).
    pub lag_ms: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Progress {
    /// Seconds where the pair is held together and rotating.
    pub held_seconds: f64,
    /// Smallest / largest singular value of the rotation-axis distribution, 0..=1. Low values
    /// mean the user only rotated about one axis.
    pub rotation_diversity: f64,
    /// Enough data to attempt [`solve`].
    pub ready: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Solution {
    pub controller: String,
    pub tracker: String,
    /// X: p_steamvr = R(rotation) · p_map + translation.
    pub rotation: Quat,
    pub translation: [f64; 3],
    /// A tracker sample at re-timed time τ matches SteamVR's pose at τ − lag.
    pub lag_ms: f64,
    pub rms_mm: f64,
    pub rot_median_deg: f64,
    pub samples: usize,
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("not enough data: {0}")]
    NotEnoughData(String),
    #[error("calibration did not converge: {0}")]
    Diverged(String),
}

/// Re-times one tracker's samples in place (sorted by time).
///
/// Poses whose timestamps differ by <= 2 ms form a batch; a batch of n poses is spread evenly over
/// (previous batch time, this batch time], the newest keeping its timestamp. The first batch and
/// batches after a dropout (> 70 ms) use the nominal 122 Hz spacing. Causal, so it can run online.
pub fn retime_batches(samples: &mut [Sample]) {
    track::retime(samples)
}

/// Detection uses at most the most recent this many seconds.
pub const DETECT_WINDOW_S: f64 = 60.0;
/// [`Progress::ready`] thresholds. fig8-1 (prototype): 39 s held, diversity 0.76 after 60 s; the
/// solve is already close with ~15–20 s of figure eights, but live sessions are noisier (faster
/// motion, looser grip), so ask for 25 s, close to HTC's Windows calibrator (~30 s).
pub const READY_HELD_S: f64 = 25.0;
pub const READY_DIVERSITY: f64 = 0.5;
/// Minimum synchronized controller samples for [`solve`].
pub const MIN_SAMPLES: usize = 200;

/// All controller × tracker pairs, best match first.
///
/// Pairs with less than 5 s of overlapping data (or no motion at all) are omitted. Callers should
/// require a high correlation (> ~0.9) and a clear margin over the runner-up before trusting it.
pub fn detect(rec: &Recording) -> Vec<Detection> {
    let Some(t_ref) = t_ref(rec.controllers.values().chain(rec.trackers.values())) else {
        return Vec::new();
    };
    let ctrls: Vec<(&String, Track)> = rec
        .controllers
        .iter()
        .map(|(k, v)| (k, Track::new(v, t_ref)))
        .filter(|(_, t)| t.len() >= 2)
        .collect();
    let trks: Vec<(&String, Track)> = rec
        .trackers
        .iter()
        .map(|(k, v)| (k, Track::new(v, t_ref)))
        .filter(|(_, t)| t.len() >= 2)
        .collect();
    let all: Vec<&Track> = ctrls.iter().chain(&trks).map(|(_, t)| t).collect();
    let Some(full) = detect::Grid::spanning(&all) else {
        return Vec::new();
    };
    let end = full.time(full.n);
    let grid = detect::Grid::between(full.t0.max(end - DETECT_WINDOW_S), end).unwrap_or(full);
    let wc: Vec<Vec<f64>> = ctrls
        .iter()
        .map(|(_, t)| detect::angular_speed(t, &grid))
        .collect();
    let wt: Vec<Vec<f64>> = trks
        .iter()
        .map(|(_, t)| detect::angular_speed(t, &grid))
        .collect();
    let mut out = Vec::new();
    for (ci, (c, _)) in ctrls.iter().enumerate() {
        for (ti, (t, _)) in trks.iter().enumerate() {
            if let Some((r, lag)) = detect::best_lag(&wc[ci], &wt[ti]) {
                out.push(Detection {
                    controller: (*c).clone(),
                    tracker: (*t).clone(),
                    correlation: r,
                    lag_ms: lag * 1000.0,
                });
            }
        }
    }
    out.sort_by(|a, b| b.correlation.total_cmp(&a.correlation));
    out
}

/// Held-and-moving data of one pair, shared by [`progress`] and [`solve`].
struct Pair {
    ctrl: Track,
    trk: Track,
    held_seconds: f64,
    /// Controller sample indices inside held-and-moving segments.
    selected: Vec<usize>,
}

fn pair(rec: &Recording, det: &Detection) -> Result<Pair, Error> {
    let c = rec.controllers.get(&det.controller).ok_or_else(|| {
        Error::NotEnoughData(format!("no samples of controller {}", det.controller))
    })?;
    let t = rec
        .trackers
        .get(&det.tracker)
        .ok_or_else(|| Error::NotEnoughData(format!("no samples of tracker {}", det.tracker)))?;
    let t_ref = t_ref([c, t]).ok_or_else(|| Error::NotEnoughData("empty recording".into()))?;
    let (ctrl, trk) = (Track::new(c, t_ref), Track::new(t, t_ref));
    let none = || Error::NotEnoughData("recording too short".into());
    if ctrl.len() < 2 || trk.len() < 2 {
        return Err(none());
    }
    let grid = detect::Grid::spanning(&[&ctrl, &trk]).ok_or_else(none)?;
    let wc = detect::angular_speed(&ctrl, &grid);
    let wt = detect::shifted(
        &detect::angular_speed(&trk, &grid),
        (det.lag_ms / 1000.0 * detect::GRID_HZ).round() as i64,
    );
    let mask = detect::held_mask(&wc, &wt);
    let held_seconds = mask.iter().filter(|&&m| m).count() as f64 / detect::GRID_HZ;
    let selected = (0..ctrl.len())
        .filter(|&i| grid.index(ctrl.t[i]).is_some_and(|g| mask[g]))
        .collect();
    Ok(Pair {
        ctrl,
        trk,
        held_seconds,
        selected,
    })
}

fn t_ref<'a>(tracks: impl IntoIterator<Item = &'a Vec<Sample>>) -> Option<i64> {
    tracks
        .into_iter()
        .filter_map(|v| v.iter().map(|s| s.t_ns).min())
        .min()
}

/// Rotation diversity is computed from the controller's relative rotations ~0.3 s apart inside
/// held-and-moving segments (same axes as the solver's rotation initialization).
pub fn progress(rec: &Recording, det: &Detection) -> Progress {
    let Ok(p) = pair(rec, det) else {
        return Progress {
            held_seconds: 0.0,
            rotation_diversity: 0.0,
            ready: false,
        };
    };
    let tc: Vec<f64> = p.selected.iter().map(|&i| p.ctrl.t[i]).collect();
    let rc: Vec<_> = p
        .selected
        .iter()
        .map(|&i| math::quat_to_mat(&p.ctrl.q[i]))
        .collect();
    let (axes, _, _) = detect::rotation_axes(&tc, &rc);
    let rotation_diversity = detect::axis_diversity(&axes);
    Progress {
        held_seconds: p.held_seconds,
        rotation_diversity,
        ready: p.held_seconds >= READY_HELD_S
            && rotation_diversity >= READY_DIVERSITY
            && p.selected.len() >= MIN_SAMPLES,
    }
}

/// Options for [`solve_with`].
#[derive(Debug, Clone, Copy)]
pub struct SolveOptions {
    /// Let Y drift linearly in time (6 nuisance parameters). fig8-1 shows a steady 0.27 °/s
    /// relative rotation about the controller's z axis that otherwise biases X.
    pub drift: bool,
    pub iterations: usize,
}

impl Default for SolveOptions {
    fn default() -> Self {
        SolveOptions {
            drift: true,
            iterations: 30,
        }
    }
}

pub fn solve(rec: &Recording, det: &Detection) -> Result<Solution, Error> {
    solve_with(rec, det, &SolveOptions::default())
}

pub fn solve_with(rec: &Recording, det: &Detection, opt: &SolveOptions) -> Result<Solution, Error> {
    let p = pair(rec, det)?;
    let lag0 = det.lag_ms / 1000.0;
    // The tracker must be interpolable within lag +- 50 ms.
    let ok = |t: f64| {
        [-0.05, 0.0, 0.05]
            .iter()
            .all(|dl| p.trk.sample(t + lag0 + dl, 0.05).2)
    };
    let sel: Vec<usize> = p
        .selected
        .iter()
        .copied()
        .filter(|&i| ok(p.ctrl.t[i]))
        .collect();
    if sel.len() < MIN_SAMPLES {
        return Err(Error::NotEnoughData(format!(
            "{} synchronized samples ({:.1} s held and moving), need {MIN_SAMPLES}",
            sel.len(),
            p.held_seconds
        )));
    }
    let d = solver::Data::new(
        &p.trk,
        sel.iter().map(|&i| p.ctrl.t[i]).collect(),
        sel.iter().map(|&i| p.ctrl.p[i]).collect(),
        sel.iter()
            .map(|&i| math::quat_to_mat(&p.ctrl.q[i]))
            .collect(),
    );
    let init = solver::pick_lag(&d, lag0)
        .ok_or_else(|| Error::NotEnoughData("not enough rotation to initialize".into()))?;
    let s = solver::refine(&d, &init.state, opt.drift, opt.iterations);
    let (ep, mut er) = solver::stats(&d, &s);
    let rms_mm = solver::rms(&ep);
    let rot_median_deg = math::median(&mut er);
    let finite = rms_mm.is_finite()
        && rot_median_deg.is_finite()
        && s.tx.iter().all(|v| v.is_finite())
        && s.rx.iter().all(|v| v.is_finite());
    if !finite || (s.lag - lag0).abs() > 0.1 {
        return Err(Error::Diverged(format!(
            "rms {rms_mm:.1} mm, lag {:.1} ms",
            s.lag * 1000.0
        )));
    }
    Ok(Solution {
        controller: det.controller.clone(),
        tracker: det.tracker.clone(),
        rotation: math::mat_to_quat(&s.rx),
        translation: s.tx.into(),
        lag_ms: s.lag * 1000.0,
        rms_mm,
        rot_median_deg,
        samples: d.tc.len(),
    })
}

/// Yaw (degrees, about +Y) of a rotation; for yaw-only rotations this is the full rotation.
pub fn yaw_deg(q: &Quat) -> f64 {
    math::yaw_deg(&math::quat_to_mat(q))
}

/// Loads a `utk record --steamvr` JSON Lines file: tracker poses (`src: "vut"`, timestamp `ts`,
/// re-timed with [`retime_batches`]) and valid SteamVR controller poses (`t`). Keys are serials.
pub fn load_jsonl(path: impl AsRef<std::path::Path>) -> std::io::Result<Recording> {
    use std::io::BufRead;
    let f = std::io::BufReader::new(std::fs::File::open(path)?);
    let mut rec = Recording::default();
    let arr = |v: &serde_json::Value| -> Option<Vec<f64>> {
        v.as_array()?.iter().map(|x| x.as_f64()).collect()
    };
    for line in f.lines() {
        let line = line?;
        let Ok(r) = serde_json::from_str::<serde_json::Value>(&line) else {
            continue;
        };
        let (Some(p), Some(q)) = (arr(&r["p"]), arr(&r["q"])) else {
            continue;
        };
        if p.len() != 3 || q.len() != 4 {
            continue;
        }
        let serial = r["serial"].as_str().unwrap_or_default().to_owned();
        let (map, t) = match r["src"].as_str() {
            Some("vut") => (&mut rec.trackers, r["ts"].as_i64()),
            Some("steamvr")
                if r["class"] == "controller"
                    && r["valid"] == true
                    && r["result"].as_i64() == Some(200) =>
            {
                (&mut rec.controllers, r["t"].as_i64())
            }
            _ => continue,
        };
        let Some(t_ns) = t else { continue };
        map.entry(serial).or_default().push(Sample {
            t_ns,
            pos: [p[0], p[1], p[2]],
            rot: [q[0], q[1], q[2], q[3]],
        });
    }
    for v in rec.controllers.values_mut() {
        v.sort_by_key(|s| s.t_ns);
    }
    for v in rec.trackers.values_mut() {
        v.sort_by_key(|s| s.t_ns);
        retime_batches(v);
    }
    Ok(rec)
}

impl Solution {
    /// calibration.json content read by the SteamVR driver.
    pub fn to_json(&self, created_unix: u64) -> serde_json::Value {
        serde_json::json!({
            "version": 1,
            "tracker_serial": self.tracker,
            "controller_serial": self.controller,
            "rotation": self.rotation,
            "translation": self.translation,
            "lag_ms": self.lag_ms,
            "rms_mm": self.rms_mm,
            "rot_median_deg": self.rot_median_deg,
            "created_unix": created_unix,
        })
    }
}
