//! Detection of the held pair and of held-and-moving segments via angular-speed series.

use crate::math::M3;
use crate::track::Track;

pub const GRID_HZ: f64 = 120.0;
/// Angular speed from poses this far apart: frame-to-frame speed drowns in jitter (corr 0.19 at
/// 17 ms), over 150 ms the correlation of a held pair is > 0.97.
const SPEED_DT: f64 = 0.15;
/// Lag search range in grid steps (+-300 ms).
const MAX_LAG_STEPS: i64 = 36;
/// Minimum overlap for a correlation (5 s).
const MIN_CORR_SAMPLES: usize = (GRID_HZ * 5.0) as usize;

/// Held-mask windows: 2 s, hop 0.5 s, corr > 0.8, controller mean speed >= 0.5 rad/s.
const HELD_WIN_S: f64 = 2.0;
const HELD_MIN_CORR: f64 = 0.8;
const HELD_MIN_SPEED: f64 = 0.5;

/// Uniform time grid (seconds, same timeline as the tracks).
pub struct Grid {
    pub t0: f64,
    pub n: usize,
}

impl Grid {
    /// Grid over [t_min + 0.5, t_max - 0.5].
    pub fn spanning(tracks: &[&Track]) -> Option<Grid> {
        let lo = tracks
            .iter()
            .filter(|t| t.len() > 0)
            .map(|t| t.t[0])
            .fold(f64::INFINITY, f64::min);
        let hi = tracks
            .iter()
            .filter(|t| t.len() > 0)
            .map(|t| t.t[t.len() - 1])
            .fold(f64::NEG_INFINITY, f64::max);
        Self::between(lo + 0.5, hi - 0.5)
    }

    pub fn between(lo: f64, hi: f64) -> Option<Grid> {
        if hi.partial_cmp(&lo) != Some(std::cmp::Ordering::Greater) {
            return None;
        }
        Some(Grid {
            t0: lo,
            n: ((hi - lo) * GRID_HZ).ceil() as usize,
        })
    }

    pub fn time(&self, i: usize) -> f64 {
        self.t0 + i as f64 / GRID_HZ
    }

    /// Nearest grid index of `t`, if inside.
    pub fn index(&self, t: f64) -> Option<usize> {
        let i = ((t - self.t0) * GRID_HZ).round();
        (i >= 0.0 && (i as usize) < self.n).then_some(i as usize)
    }
}

/// Angular speed (rad/s) on the grid from poses at t +- SPEED_DT/2; NaN where not interpolable.
pub fn angular_speed(tk: &Track, g: &Grid) -> Vec<f64> {
    (0..g.n)
        .map(|i| {
            let t = g.time(i);
            let (_, qa, va) = tk.sample(t - SPEED_DT / 2.0, 0.05);
            let (_, qb, vb) = tk.sample(t + SPEED_DT / 2.0, 0.05);
            if !(va && vb) {
                return f64::NAN;
            }
            let d = (0..4)
                .map(|k| qa[k] * qb[k])
                .sum::<f64>()
                .abs()
                .clamp(0.0, 1.0);
            2.0 * d.acos() / SPEED_DT
        })
        .collect()
}

/// Pearson correlation of a[i] and b[i + shift] over the indices where both are finite.
/// Returns (corr, count); corr is NaN for too little overlap or no variation.
pub fn corr_shifted(a: &[f64], b: &[f64], shift: i64, min_n: usize) -> (f64, usize) {
    let n = a.len().min(b.len()) as i64;
    let (lo, hi) = ((-shift).max(0), (n - shift).min(n));
    let (mut cnt, mut sa, mut sb, mut saa, mut sbb, mut sab) = (0usize, 0.0, 0.0, 0.0, 0.0, 0.0);
    for i in lo..hi {
        let (x, y) = (a[i as usize], b[(i + shift) as usize]);
        if x.is_nan() || y.is_nan() {
            continue;
        }
        cnt += 1;
        sa += x;
        sb += y;
        saa += x * x;
        sbb += y * y;
        sab += x * y;
    }
    if cnt < min_n.max(2) {
        return (f64::NAN, cnt);
    }
    let c = cnt as f64;
    let va = saa / c - (sa / c).powi(2);
    let vb = sbb / c - (sb / c).powi(2);
    if va < 1e-12 || vb < 1e-12 {
        return (f64::NAN, cnt);
    }
    ((sab / c - sa * sb / (c * c)) / (va * vb).sqrt(), cnt)
}

/// Best (corr, lag in seconds) over the lag search range.
pub fn best_lag(wc: &[f64], wt: &[f64]) -> Option<(f64, f64)> {
    let mut best: Option<(f64, i64)> = None;
    for lag in -MAX_LAG_STEPS..=MAX_LAG_STEPS {
        let (r, _) = corr_shifted(wc, wt, lag, MIN_CORR_SAMPLES);
        if !r.is_nan() && best.is_none_or(|(b, _)| r > b) {
            best = Some((r, lag));
        }
    }
    best.map(|(r, l)| (r, l as f64 / GRID_HZ))
}

/// Grid mask of "held together and rotating": windowed speed correlation > 0.8 and mean
/// controller speed >= 0.5 rad/s. `wt` must already be shifted by the lag.
pub fn held_mask(wc: &[f64], wt: &[f64]) -> Vec<bool> {
    let n = (HELD_WIN_S * GRID_HZ) as usize;
    let len = wc.len().min(wt.len());
    let mut mask = vec![false; len];
    let mut s = 0;
    while s + n <= len {
        let (a, b) = (&wc[s..s + n], &wt[s..s + n]);
        let (mut na, mut suma) = (0usize, 0.0);
        for &x in a.iter().filter(|x| !x.is_nan()) {
            na += 1;
            suma += x;
        }
        let mean_a = suma / na.max(1) as f64;
        let (r, cnt) = corr_both(a, b);
        if cnt >= n / 2 && na > 0 && mean_a >= HELD_MIN_SPEED && r > HELD_MIN_CORR {
            mask[s..s + n].iter_mut().for_each(|m| *m = true);
        }
        s += n / 4;
    }
    mask
}

/// Correlation over jointly finite entries; NaN when either side has std < 1e-3.
fn corr_both(a: &[f64], b: &[f64]) -> (f64, usize) {
    let (mut c, mut sa, mut sb, mut saa, mut sbb, mut sab) = (0usize, 0.0, 0.0, 0.0, 0.0, 0.0);
    for (&x, &y) in a.iter().zip(b) {
        if x.is_nan() || y.is_nan() {
            continue;
        }
        c += 1;
        sa += x;
        sb += y;
        saa += x * x;
        sbb += y * y;
        sab += x * y;
    }
    if c < 2 {
        return (f64::NAN, c);
    }
    let n = c as f64;
    let va = (saa / n - (sa / n).powi(2)).max(0.0);
    let vb = (sbb / n - (sb / n).powi(2)).max(0.0);
    if va.sqrt() < 1e-3 || vb.sqrt() < 1e-3 {
        return (f64::NAN, c);
    }
    ((sab / n - sa * sb / (n * n)) / (va * vb).sqrt(), c)
}

/// Shift `w` so that out[i] = w[i + shift] (NaN outside).
pub fn shifted(w: &[f64], shift: i64) -> Vec<f64> {
    (0..w.len() as i64)
        .map(|i| {
            let j = i + shift;
            if j >= 0 && j < w.len() as i64 {
                w[j as usize]
            } else {
                f64::NAN
            }
        })
        .collect()
}

/// Unit rotation axes of relative rotations ~0.3 s apart (angle > 0.4 rad), in the frame of
/// `rots` (world-frame axes: R_{i+k} R_i^T). Indices are pairs (i, i + step).
pub fn rotation_axes(t: &[f64], rots: &[M3]) -> (Vec<nalgebra::Vector3<f64>>, Vec<usize>, usize) {
    let mut diffs: Vec<f64> = t
        .windows(2)
        .map(|w| w[1] - w[0])
        .filter(|&d| d > 0.0)
        .collect();
    let md = crate::math::median(&mut diffs);
    let step = if md.is_finite() && md > 0.0 {
        ((0.3 / md).round() as usize).max(1)
    } else {
        1
    };
    let mut axes = Vec::new();
    let mut idx = Vec::new();
    for i in 0..t.len().saturating_sub(step) {
        if t[i + step] - t[i] >= 0.4 {
            continue;
        }
        let d = rots[i + step] * rots[i].transpose();
        if crate::math::angle(&d) > 0.4 {
            axes.push(crate::math::log_so3(&d).normalize());
            idx.push(i);
        }
    }
    (axes, idx, step)
}

/// Smallest / largest singular value of the stacked unit axes (0 when none).
pub fn axis_diversity(axes: &[nalgebra::Vector3<f64>]) -> f64 {
    if axes.len() < 3 {
        return 0.0;
    }
    let mut m = M3::zeros();
    for a in axes {
        m += a * a.transpose();
    }
    let ev = m.symmetric_eigenvalues();
    let (lo, hi) = (ev.min().max(0.0), ev.max());
    if hi > 0.0 { (lo / hi).sqrt() } else { 0.0 }
}
