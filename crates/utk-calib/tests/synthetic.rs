//! Synthetic recording with known X, Y and lag.

use nalgebra::{Matrix3, Rotation3, Vector3};
use utk_calib::{Recording, Sample, detect, progress, retime_batches, solve, yaw_deg};

type M3 = Matrix3<f64>;
type V3 = Vector3<f64>;

fn quat(m: &M3) -> [f64; 4] {
    let q = nalgebra::UnitQuaternion::from_matrix(m);
    [q.i, q.j, q.k, q.w]
}

/// Controller pose at time t (s): figure eight with rotation about all axes.
fn ctrl_pose(t: f64) -> (V3, M3) {
    let p = V3::new(
        0.4 * (1.1 * t).sin(),
        1.2 + 0.2 * (2.2 * t).sin(),
        0.3 * (0.7 * t).cos(),
    );
    let v = V3::new(
        1.2 * (1.3 * t).sin(),
        1.5 * (0.9 * t + 1.0).sin(),
        0.9 * (1.7 * t + 2.0).sin(),
    );
    (p, *Rotation3::from_scaled_axis(v).matrix())
}

#[test]
fn recovers_known_calibration() {
    let yaw = 1.234;
    let rx = *Rotation3::from_axis_angle(&V3::y_axis(), yaw).matrix();
    let tx = V3::new(1.3, 1.1, -0.4);
    let ry = *Rotation3::from_scaled_axis(V3::new(0.4, -0.8, 0.3)).matrix();
    let y = V3::new(-0.08, 0.08, 0.04);
    let lag = -0.006; // C(t) = X T(t + lag) Y
    let dur = 35.0; // longer than READY_HELD_S plus the edges trimmed by the held mask
    let mut rec = Recording::default();

    let ctrl: Vec<Sample> = (0..(dur * 90.0) as i64)
        .map(|i| {
            let t = i as f64 / 90.0;
            let (p, r) = ctrl_pose(t);
            Sample {
                t_ns: (t * 1e9) as i64 + 1_000_000_000,
                pos: p.into(),
                rot: quat(&r),
            }
        })
        .collect();
    rec.controllers.insert("ctrl".into(), ctrl.clone());
    // A static second controller.
    rec.controllers.insert(
        "idle".into(),
        ctrl.iter()
            .map(|s| Sample {
                pos: [0.0, 1.0, 0.0],
                rot: [0.0, 0.0, 0.0, 1.0],
                ..*s
            })
            .collect(),
    );

    // Tracker: true sample times at 122 Hz, delivered in 50 ms batches whose poses are all
    // stamped (µs apart) with the time of the batch's newest pose.
    let (rxi, ryi) = (rx.transpose(), ry.transpose());
    let mut trk = Vec::new();
    let mut decoy = Vec::new();
    let n = (dur * 122.0) as i64;
    let batch = |i: i64| ((i as f64 / 122.0) / 0.05).floor() as i64;
    for i in 0..n {
        let tau = i as f64 / 122.0;
        let last = (i..n).take_while(|&j| batch(j) == batch(i)).last().unwrap();
        // T(tau) = X^-1 C(tau - lag) Y^-1
        let (pc, rc) = ctrl_pose(tau - lag);
        let rt = rxi * rc * ryi;
        let pt = rxi * (pc - tx) - rt * y;
        let stamp = (last as f64 / 122.0 * 1e9) as i64 + 1_000_000_000 - (last - i) * 1000;
        trk.push(Sample {
            t_ns: stamp,
            pos: pt.into(),
            rot: quat(&rt),
        });
        let wobble =
            *Rotation3::from_scaled_axis(V3::new(0.0, 0.3 * (0.5 * tau).sin(), 0.0)).matrix();
        decoy.push(Sample {
            t_ns: stamp,
            pos: [0.0, 0.0, 0.0],
            rot: quat(&wobble),
        });
    }
    retime_batches(&mut trk);
    retime_batches(&mut decoy);
    rec.trackers.insert("trk".into(), trk);
    rec.trackers.insert("decoy".into(), decoy);

    let dets = detect(&rec);
    let best = &dets[0];
    assert_eq!(
        (best.controller.as_str(), best.tracker.as_str()),
        ("ctrl", "trk")
    );
    assert!(best.correlation > 0.95);
    let prog = progress(&rec, best);
    assert!(prog.ready, "{prog:?}");
    let sol = solve(&rec, best).unwrap();
    let dyaw = (yaw_deg(&sol.rotation) - yaw.to_degrees() + 540.0) % 360.0 - 180.0;
    let dt = (V3::from(sol.translation) - tx).norm();
    assert!(dyaw.abs() < 0.2, "yaw error {dyaw}");
    assert!(dt < 0.005, "translation error {dt}");
    assert!(
        (sol.lag_ms - lag * 1000.0).abs() < 1.0,
        "lag {}",
        sol.lag_ms
    );
    assert!(sol.rms_mm < 2.0, "{sol:?}");
    eprintln!("{sol:?} yaw err {dyaw:.4} deg, t err {:.2} mm", dt * 1000.0);
}
