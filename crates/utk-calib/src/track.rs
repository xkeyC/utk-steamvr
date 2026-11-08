//! Pose tracks on a common f64 timeline, interpolation and batch re-timing.

use crate::Sample;
use crate::math::{M3, V3, quat_to_mat};

/// Tracker_server batch: poses whose `ts` differ by less than this belong to one batch.
const BATCH_GAP_NS: i64 = 2_000_000;
/// Longer batch intervals are dropouts; use the nominal rate instead.
const MAX_SPAN_NS: i64 = 70_000_000;
const NOMINAL_HZ: f64 = 122.0;

pub fn retime(samples: &mut [Sample]) {
    let n = samples.len();
    let mut start = 0;
    let mut prev_end: Option<i64> = None;
    for e in 0..n {
        // Batch boundaries are tested on original timestamps: e + 1 is not re-timed yet and the
        // newest pose of each batch keeps its value.
        if e + 1 < n && samples[e + 1].t_ns - samples[e].t_ns <= BATCH_GAP_NS {
            continue;
        }
        let k = (e - start + 1) as i64;
        let end = samples[e].t_ns;
        let nominal = (k as f64 / NOMINAL_HZ * 1e9).round() as i64;
        let span = match prev_end {
            Some(p) if end - p > 0 && end - p <= MAX_SPAN_NS => end - p,
            _ => nominal,
        };
        for j in 0..k {
            let off = (span as f64 * (k - 1 - j) as f64 / k as f64).round() as i64;
            samples[start + j as usize].t_ns = end - off;
        }
        prev_end = Some(end);
        start = e + 1;
    }
}

/// One device's poses, sorted, quaternion signs continuous. Times in seconds relative to `t_ref`.
pub struct Track {
    pub t: Vec<f64>,
    pub p: Vec<V3>,
    pub q: Vec<[f64; 4]>,
}

impl Track {
    pub fn new(samples: &[Sample], t_ref: i64) -> Track {
        let mut s: Vec<&Sample> = samples.iter().collect();
        if !s.is_sorted_by_key(|x| x.t_ns) {
            s.sort_by_key(|x| x.t_ns);
        }
        let t = s.iter().map(|x| (x.t_ns - t_ref) as f64 * 1e-9).collect();
        let p = s.iter().map(|x| V3::from(x.pos)).collect();
        let mut q: Vec<[f64; 4]> = Vec::with_capacity(s.len());
        for x in &s {
            let mut r = x.rot;
            if let Some(prev) = q.last()
                && (0..4).map(|i| r[i] * prev[i]).sum::<f64>() < 0.0
            {
                r = r.map(|v| -v);
            }
            q.push(r);
        }
        Track { t, p, q }
    }

    pub fn len(&self) -> usize {
        self.t.len()
    }

    /// Linear position / nlerp rotation at `t`. Returns `valid = false` outside the track or
    /// across gaps > `max_gap`; the (extrapolated) pose is returned anyway, like the prototype.
    pub fn sample(&self, t: f64, max_gap: f64) -> (V3, [f64; 4], bool) {
        let n = self.t.len();
        if n < 2 {
            return match n {
                1 => (self.p[0], self.q[0], false),
                _ => (V3::zeros(), [0.0, 0.0, 0.0, 1.0], false),
            };
        }
        let i = self.t.partition_point(|&x| x < t).clamp(1, n - 1);
        let (t0, t1) = (self.t[i - 1], self.t[i]);
        let dt = t1 - t0;
        let valid = t >= self.t[0] && t <= self.t[n - 1] && dt <= max_gap && dt > 0.0;
        let a = if dt > 0.0 { (t - t0) / dt } else { 0.0 };
        let p = self.p[i - 1] + (self.p[i] - self.p[i - 1]) * a;
        let (qa, qb) = (self.q[i - 1], self.q[i]);
        let mut q = [0.0; 4];
        for k in 0..4 {
            q[k] = qa[k] + a * (qb[k] - qa[k]);
        }
        let nrm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
        (p, q.map(|v| v / nrm), valid)
    }

    pub fn sample_mat(&self, t: f64, max_gap: f64) -> (V3, M3, bool) {
        let (p, q, ok) = self.sample(t, max_gap);
        (p, quat_to_mat(&q), ok)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(t_ns: i64) -> Sample {
        Sample {
            t_ns,
            pos: [0.0; 3],
            rot: [0.0, 0.0, 0.0, 1.0],
        }
    }

    #[test]
    fn retime_spreads_batches() {
        const MS: i64 = 1_000_000;
        // Batch 1 (2 poses), batch 2 50 ms later (5 poses, µs apart), then a 200 ms dropout (2 poses).
        let raw = [
            0,
            10_000,
            50 * MS,
            50 * MS + 3_000,
            50 * MS + 6_000,
            50 * MS + 9_000,
            50 * MS + 12_000,
            250 * MS,
            250 * MS + 5_000,
        ];
        let mut v: Vec<Sample> = raw.iter().map(|&t| s(t)).collect();
        retime(&mut v);
        let t: Vec<i64> = v.iter().map(|x| x.t_ns).collect();
        let nominal = (1e9 / 122.0) as i64;
        // First batch: nominal spacing, newest keeps its ts.
        assert_eq!(t[1], 10_000);
        assert!((t[1] - t[0] - nominal).abs() <= 1);
        // Second batch: spread over (10 µs, 50.012 ms], newest keeps its ts.
        let end = 50 * MS + 12_000;
        assert_eq!(t[6], end);
        let step = (end - 10_000) / 5;
        for j in 0..5 {
            assert!(
                (t[2 + j] - (10_000 + step * (j as i64 + 1))).abs() <= 1,
                "{t:?}"
            );
        }
        // After the dropout: nominal spacing again.
        assert_eq!(t[8], 250 * MS + 5_000);
        assert!((t[8] - t[7] - nominal).abs() <= 1);
        assert!(t.windows(2).all(|w| w[1] > w[0]));
    }

    #[test]
    fn interpolation_and_gaps() {
        let mut v: Vec<Sample> = (0..3).map(|i| s(i * 10_000_000)).collect();
        v[2].t_ns = 200_000_000; // 190 ms gap
        v[1].pos = [1.0, 0.0, 0.0];
        let tk = Track::new(&v, 0);
        let (p, _, ok) = tk.sample(0.005, 0.05);
        assert!(ok && (p.x - 0.5).abs() < 1e-12);
        assert!(!tk.sample(0.1, 0.05).2);
        assert!(!tk.sample(-0.001, 0.05).2);
    }
}
