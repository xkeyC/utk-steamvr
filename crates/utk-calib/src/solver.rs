//! Closed-form initial solve and Gauss-Newton/Huber refinement of C_i = X T(t_i + lag) Y.

use nalgebra::{DMatrix, DVector, Matrix6, Vector6};

use crate::detect::rotation_axes;
use crate::math::{M3, V3, exp_so3, log_so3, median, project_rotation, rot_y, yaw_kabsch};
use crate::track::Track;

const SIGMA_P: f64 = 0.010; // m
const SIGMA_R: f64 = 1.5 * std::f64::consts::PI / 180.0; // rad
const HUBER_K: f64 = 3.0;

/// Fixed controller samples (native timestamps); the tracker is interpolated at t + lag.
pub struct Data<'a> {
    pub trk: &'a Track,
    pub tc: Vec<f64>,
    pub pc: Vec<V3>,
    pub rc: Vec<M3>,
    /// tc - mean(tc), for the Y drift terms.
    pub dt: Vec<f64>,
}

impl<'a> Data<'a> {
    pub fn new(trk: &'a Track, tc: Vec<f64>, pc: Vec<V3>, rc: Vec<M3>) -> Self {
        let mean = tc.iter().sum::<f64>() / tc.len().max(1) as f64;
        let dt = tc.iter().map(|t| t - mean).collect();
        Data {
            trk,
            tc,
            pc,
            rc,
            dt,
        }
    }

    pub fn tracker(&self, lag: f64) -> (Vec<V3>, Vec<M3>) {
        self.tc
            .iter()
            .map(|&t| {
                let (p, r, _) = self.trk.sample_mat(t + lag, 0.05);
                (p, r)
            })
            .unzip()
    }
}

#[derive(Clone, Debug)]
pub struct State {
    pub rx: M3,
    pub ry: M3,
    pub y: V3,
    pub tx: V3,
    /// Seconds.
    pub lag: f64,
    /// Linear drift of Y (rad/s in the controller frame, m/s in the tracker frame); None = off.
    pub drift: Option<(V3, V3)>,
}

/// Position residual (m, SteamVR frame) and rotation residual (rotation vector, rad, controller
/// frame) of sample i, given the tracker pose at tc_i + lag.
#[inline]
fn residual(d: &Data, s: &State, i: usize, pt: &V3, rt: &M3) -> (V3, V3) {
    let (ry, off) = match &s.drift {
        Some((wy, vy)) => {
            let dt = d.dt[i];
            (s.ry * exp_so3(&(wy * dt)), rt * (s.y + vy * dt))
        }
        None => (s.ry, rt * s.y),
    };
    let rp = s.rx * (pt + off) + s.tx - d.pc[i];
    let rr = log_so3(&((s.rx * rt * ry).transpose() * d.rc[i]));
    (rp, rr)
}

/// Per-sample position error (mm) and rotation error (deg).
pub fn stats(d: &Data, s: &State) -> (Vec<f64>, Vec<f64>) {
    let (pt, rt) = d.tracker(s.lag);
    (0..d.tc.len())
        .map(|i| {
            let (rp, rr) = residual(d, s, i, &pt[i], &rt[i]);
            (rp.norm() * 1000.0, rr.norm().to_degrees())
        })
        .unzip()
}

pub fn rms(x: &[f64]) -> f64 {
    (x.iter().map(|v| v * v).sum::<f64>() / x.len().max(1) as f64).sqrt()
}

pub struct Initial {
    pub state: State,
}

/// Axis Kabsch (yaw only) for R_x, chordal mean for R_y, linear least squares for (y, t_x) with
/// outlier rejection.
pub fn initial_solve(d: &Data, lag: f64) -> Option<Initial> {
    let (pt, rt) = d.tracker(lag);
    let (ac, idx, step) = rotation_axes(&d.tc, &d.rc);
    if ac.len() < 10 {
        return None;
    }
    let at: Vec<V3> = idx
        .iter()
        .map(|&i| log_so3(&(rt[i + step] * rt[i].transpose())).normalize())
        .collect();
    let rx = yaw_kabsch(&at, &ac);

    let mut m = M3::zeros();
    for (r, c) in rt.iter().zip(&d.rc) {
        m += (rx * r).transpose() * c;
    }
    let ry = project_rotation(&m);

    // [R_x R_t_i, I] [y; t_x] = p_c_i - R_x p_t_i
    let a: Vec<M3> = rt.iter().map(|r| rx * r).collect();
    let b: Vec<V3> = (0..d.tc.len()).map(|i| d.pc[i] - rx * pt[i]).collect();
    let n = d.tc.len();
    let mut keep = vec![true; n];
    let mut sol = Vector6::zeros();
    for _ in 0..3 {
        let mut ata = Matrix6::zeros();
        let mut atb = Vector6::zeros();
        for i in (0..n).filter(|&i| keep[i]) {
            let ai = a[i];
            // A_i^T A_i = [[I, R^T], [R, I]] for a rotation R.
            let mut blk = Matrix6::identity();
            blk.fixed_view_mut::<3, 3>(0, 3).copy_from(&ai.transpose());
            blk.fixed_view_mut::<3, 3>(3, 0).copy_from(&ai);
            ata += blk;
            let atb_y = ai.transpose() * b[i];
            for k in 0..3 {
                atb[k] += atb_y[k];
                atb[k + 3] += b[i][k];
            }
        }
        sol = ata.svd(true, true).solve(&atb, 1e-12).ok()?;
        let (y, tx) = (
            sol.fixed_rows::<3>(0).into_owned(),
            sol.fixed_rows::<3>(3).into_owned(),
        );
        let err: Vec<f64> = (0..n).map(|i| (a[i] * y + tx - b[i]).norm()).collect();
        let mut kept: Vec<f64> = (0..n).filter(|&i| keep[i]).map(|i| err[i]).collect();
        let thr = (3.0 * median(&mut kept)).max(0.01);
        keep = err.iter().map(|&e| e < thr).collect();
    }
    Some(Initial {
        state: State {
            rx,
            ry,
            y: sol.fixed_rows::<3>(0).into_owned(),
            tx: sol.fixed_rows::<3>(3).into_owned(),
            lag,
            drift: None,
        },
    })
}

/// Coarse lag pre-search (+-30 ms, 2 ms steps) with the closed-form solve, so Gauss-Newton starts
/// near the minimum. Returns the best initial solution.
pub fn pick_lag(d: &Data, lag0: f64) -> Option<Initial> {
    let mut best: Option<(f64, Initial)> = None;
    for k in -15..=15 {
        let lag = lag0 + k as f64 * 0.002;
        let Some(init) = initial_solve(d, lag) else {
            continue;
        };
        let e = rms(&stats(d, &init.state).0);
        if e.is_finite() && best.as_ref().is_none_or(|(b, _)| e < *b) {
            best = Some((e, init));
        }
    }
    best.map(|(_, i)| i)
}

/// Gauss-Newton + Huber IRLS with a numeric Jacobian over: R_x yaw increment (1), R_y increment
/// (3), y (3), t_x (3), lag in ms (1) and, with drift, the Y drift (3 + 3).
pub fn refine(d: &Data, s0: &State, drift: bool, iters: usize) -> State {
    let mut s = s0.clone();
    s.drift = if drift {
        Some(s.drift.unwrap_or((V3::zeros(), V3::zeros())))
    } else {
        None
    };
    let npar = if drift { 17 } else { 11 };
    const LAG: usize = 10;
    let steps: Vec<f64> = (0..npar)
        .map(|k| if k == LAG { 1e-2 } else { 1e-6 })
        .collect();
    let n = d.tc.len();

    let unpack = |s: &State, x: &[f64]| -> State {
        let mut o = s.clone();
        o.rx = rot_y(x[0]) * s.rx;
        o.ry = s.ry * exp_so3(&V3::new(x[1], x[2], x[3]));
        o.y = s.y + V3::new(x[4], x[5], x[6]);
        o.tx = s.tx + V3::new(x[7], x[8], x[9]);
        o.lag = s.lag + x[LAG] * 1e-3;
        if let Some((wy, vy)) = &s.drift {
            o.drift = Some((
                wy + V3::new(x[11], x[12], x[13]),
                vy + V3::new(x[14], x[15], x[16]),
            ));
        }
        o
    };
    // Normalized 6-vector residuals for all samples.
    let rvec = |s: &State, trk: &(Vec<V3>, Vec<M3>), out: &mut Vec<[f64; 6]>| {
        out.clear();
        for i in 0..n {
            let (rp, rr) = residual(d, s, i, &trk.0[i], &trk.1[i]);
            out.push([
                rp.x / SIGMA_P,
                rp.y / SIGMA_P,
                rp.z / SIGMA_P,
                rr.x / SIGMA_R,
                rr.y / SIGMA_R,
                rr.z / SIGMA_R,
            ]);
        }
    };

    let mut r = Vec::with_capacity(n);
    let mut rk = Vec::with_capacity(n);
    let mut jac = vec![0.0; npar * n * 6]; // column-major: column k at [k * 6n ..]
    for _ in 0..iters {
        let trk = d.tracker(s.lag);
        rvec(&s, &trk, &mut r);
        let sw: Vec<f64> = r
            .iter()
            .map(|v| {
                let nrm = (v.iter().map(|x| x * x).sum::<f64>() / 2.0).sqrt();
                if nrm < HUBER_K {
                    1.0
                } else {
                    (HUBER_K / nrm.max(1e-12)).sqrt()
                }
            })
            .collect();
        for k in 0..npar {
            let mut x = vec![0.0; npar];
            x[k] = steps[k];
            let sk = unpack(&s, &x);
            if k == LAG {
                rvec(&sk, &d.tracker(sk.lag), &mut rk);
            } else {
                rvec(&sk, &trk, &mut rk);
            }
            let col = &mut jac[k * 6 * n..(k + 1) * 6 * n];
            for i in 0..n {
                for c in 0..6 {
                    col[i * 6 + c] = (rk[i][c] - r[i][c]) * sw[i] / steps[k];
                }
            }
        }
        // Normal equations J^T J dx = -J^T r (weighted).
        let mut jtj = DMatrix::<f64>::zeros(npar, npar);
        let mut jtr = DVector::<f64>::zeros(npar);
        for a in 0..npar {
            let ca = &jac[a * 6 * n..(a + 1) * 6 * n];
            let mut acc = 0.0;
            for i in 0..n {
                for c in 0..6 {
                    acc += ca[i * 6 + c] * r[i][c] * sw[i];
                }
            }
            jtr[a] = acc;
            for b in 0..=a {
                let cb = &jac[b * 6 * n..(b + 1) * 6 * n];
                let v: f64 = ca.iter().zip(cb).map(|(x, y)| x * y).sum();
                jtj[(a, b)] = v;
                jtj[(b, a)] = v;
            }
        }
        let Ok(dx) = jtj.svd(true, true).solve(&(-jtr), 1e-12) else {
            break;
        };
        let dx: Vec<f64> = dx.iter().copied().collect();
        if dx.iter().any(|v| !v.is_finite()) {
            break;
        }
        s = unpack(&s, &dx);
        if dx.iter().zip(&steps).all(|(v, st)| v.abs() < st * 0.1) {
            break;
        }
    }
    s
}
