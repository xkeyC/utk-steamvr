//! Rotation helpers (quaternions are x, y, z, w).

use nalgebra::{Matrix3, Vector3};

pub type M3 = Matrix3<f64>;
pub type V3 = Vector3<f64>;

pub fn quat_to_mat(q: &[f64; 4]) -> M3 {
    let [x, y, z, w] = *q;
    M3::new(
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y - z * w),
        2.0 * (x * z + y * w),
        2.0 * (x * y + z * w),
        1.0 - 2.0 * (x * x + z * z),
        2.0 * (y * z - x * w),
        2.0 * (x * z - y * w),
        2.0 * (y * z + x * w),
        1.0 - 2.0 * (x * x + y * y),
    )
}

pub fn mat_to_quat(m: &M3) -> [f64; 4] {
    let w = (1.0 + m.trace()).max(0.0).sqrt() / 2.0;
    let x = ((1.0 + m[(0, 0)] - m[(1, 1)] - m[(2, 2)]).max(0.0).sqrt() / 2.0)
        .copysign(m[(2, 1)] - m[(1, 2)]);
    let y = ((1.0 - m[(0, 0)] + m[(1, 1)] - m[(2, 2)]).max(0.0).sqrt() / 2.0)
        .copysign(m[(0, 2)] - m[(2, 0)]);
    let z = ((1.0 - m[(0, 0)] - m[(1, 1)] + m[(2, 2)]).max(0.0).sqrt() / 2.0)
        .copysign(m[(1, 0)] - m[(0, 1)]);
    let n = (x * x + y * y + z * z + w * w).sqrt();
    [x / n, y / n, z / n, w / n]
}

pub fn skew(v: &V3) -> M3 {
    M3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

/// Rodrigues: rotation vector -> matrix.
pub fn exp_so3(v: &V3) -> M3 {
    let th = v.norm();
    let k = skew(v);
    let (s, c) = if th > 1e-9 {
        (th.sin() / th, (1.0 - th.cos()) / (th * th))
    } else {
        (1.0, 0.5)
    };
    M3::identity() + k * s + k * k * c
}

/// Matrix -> rotation vector (accurate away from pi; residuals here are small).
pub fn log_so3(r: &M3) -> V3 {
    let th = ((r.trace() - 1.0) / 2.0).clamp(-1.0, 1.0).acos();
    let v = V3::new(
        r[(2, 1)] - r[(1, 2)],
        r[(0, 2)] - r[(2, 0)],
        r[(1, 0)] - r[(0, 1)],
    );
    let f = if th > 1e-6 {
        th / (2.0 * th.sin())
    } else {
        0.5
    };
    v * f
}

/// Rotation angle (rad).
pub fn angle(r: &M3) -> f64 {
    ((r.trace() - 1.0) / 2.0).clamp(-1.0, 1.0).acos()
}

/// Rotation about +Y (SteamVR up).
pub fn rot_y(a: f64) -> M3 {
    let (s, c) = a.sin_cos();
    M3::new(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c)
}

/// Yaw (deg) of a rotation about +Y.
pub fn yaw_deg(r: &M3) -> f64 {
    r[(0, 2)].atan2(r[(2, 2)]).to_degrees()
}

/// Kabsch restricted to rotations about Y: R_y(theta) maximizing sum b . R a (closed form).
pub fn yaw_kabsch(a: &[V3], b: &[V3]) -> M3 {
    let (mut c, mut s) = (0.0, 0.0);
    for (a, b) in a.iter().zip(b) {
        c += b.x * a.x + b.z * a.z;
        s += b.x * a.z - b.z * a.x;
    }
    rot_y(s.atan2(c))
}

/// Nearest rotation (chordal projection) of a 3x3 matrix sum.
pub fn project_rotation(m: &M3) -> M3 {
    let svd = m.svd(true, true);
    let (u, vt) = (svd.u.unwrap(), svd.v_t.unwrap());
    let d = (u * vt).determinant().signum();
    u * M3::from_diagonal(&V3::new(1.0, 1.0, d)) * vt
}

pub fn median(v: &mut [f64]) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    v.sort_unstable_by(|a, b| a.total_cmp(b));
    let n = v.len();
    if n % 2 == 1 {
        v[n / 2]
    } else {
        (v[n / 2 - 1] + v[n / 2]) / 2.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn yaw_kabsch_recovers_yaw() {
        let r = rot_y(-2.1);
        let a: Vec<V3> = (0..50)
            .map(|i| {
                let f = i as f64;
                V3::new((f * 0.7).sin(), (f * 1.3).cos(), (f * 0.4).sin() + 0.2).normalize()
            })
            .collect();
        // Small noise on b, including a tilt the yaw-only model cannot represent.
        let b: Vec<V3> = a
            .iter()
            .enumerate()
            .map(|(i, v)| r * v + V3::new(0.0, 0.0, 1e-3 * (i % 3) as f64))
            .collect();
        let est = yaw_kabsch(&a, &b);
        assert!(
            (yaw_deg(&est) - yaw_deg(&r)).abs() < 0.2,
            "{} vs {}",
            yaw_deg(&est),
            yaw_deg(&r)
        );
        assert!((est[(1, 1)] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn so3_roundtrip() {
        let v = V3::new(0.3, -1.1, 0.7);
        assert!((log_so3(&exp_so3(&v)) - v).norm() < 1e-9);
        let m = exp_so3(&v);
        assert!((quat_to_mat(&mat_to_quat(&m)) - m).norm() < 1e-9);
    }
}
