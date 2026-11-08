//! End-to-end check on a real recording (skipped when the capture is missing).

use std::time::Instant;

use utk_calib::{SolveOptions, detect, load_jsonl, progress, solve, solve_with, yaw_deg};

const CAPTURE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../captures/fig8-1.jsonl");
const REFERENCE: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../captures/calibration-fig8-1.json"
);

#[test]
fn fig8_1() {
    let Ok(rec) = load_jsonl(CAPTURE) else {
        eprintln!("skipping: {CAPTURE} not found");
        return;
    };

    let t = Instant::now();
    let dets = detect(&rec);
    let t_detect = t.elapsed();
    for d in &dets {
        eprintln!(
            "  {} x {}: corr {:+.3} lag {:+.0} ms",
            d.controller, d.tracker, d.correlation, d.lag_ms
        );
    }
    let best = &dets[0];
    assert_eq!(best.tracker, "FA3BE3B00584");
    assert_eq!(best.controller, "VRLINKQ3_Controller_Right");
    assert!(best.correlation > 0.95, "corr {}", best.correlation);
    assert!(
        best.correlation - dets[1].correlation > 0.5,
        "margin {}",
        best.correlation - dets[1].correlation
    );

    let t = Instant::now();
    let prog = progress(&rec, best);
    let t_progress = t.elapsed();
    eprintln!("progress: {prog:?}");
    assert!(prog.ready);

    let t = Instant::now();
    let sol = solve(&rec, best).expect("solve");
    let t_solve = t.elapsed();
    eprintln!(
        "solve (drift): {sol:?}, yaw {:.2} deg",
        yaw_deg(&sol.rotation)
    );
    let nd = solve_with(
        &rec,
        best,
        &SolveOptions {
            drift: false,
            ..Default::default()
        },
    )
    .expect("solve");
    eprintln!(
        "solve (no drift): rms {:.2} mm, rot median {:.2} deg, yaw {:.2} deg",
        nd.rms_mm,
        nd.rot_median_deg,
        yaw_deg(&nd.rotation)
    );
    eprintln!("timing: detect {t_detect:.1?}, progress {t_progress:.1?}, solve {t_solve:.1?}");

    assert!(sol.rms_mm < 10.0, "rms {}", sol.rms_mm);
    assert!(sol.rot_median_deg < 2.0, "rot {}", sol.rot_median_deg);
    assert!(sol.samples > 3000);

    if let Ok(text) = std::fs::read_to_string(REFERENCE) {
        let r: serde_json::Value = serde_json::from_str(&text).unwrap();
        let q: Vec<f64> = r["rotation"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_f64().unwrap())
            .collect();
        let tr: Vec<f64> = r["translation"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_f64().unwrap())
            .collect();
        let dyaw =
            (yaw_deg(&sol.rotation) - yaw_deg(&[q[0], q[1], q[2], q[3]]) + 540.0) % 360.0 - 180.0;
        let dt = (0..3)
            .map(|i| (sol.translation[i] - tr[i]).powi(2))
            .sum::<f64>()
            .sqrt();
        eprintln!(
            "vs prototype: yaw {dyaw:+.3} deg, translation {:.1} mm, lag {:+.2} ms",
            dt * 1000.0,
            sol.lag_ms - r["lag_ms"].as_f64().unwrap()
        );
        assert!(dyaw.abs() < 1.0);
        assert!(dt < 0.01);
    }
}

/// Live-UI usage: progress over a growing recording.
#[test]
fn fig8_1_growing() {
    let Ok(rec) = load_jsonl(CAPTURE) else {
        eprintln!("skipping: {CAPTURE} not found");
        return;
    };
    let t0 = rec
        .controllers
        .values()
        .chain(rec.trackers.values())
        .filter_map(|v| v.first())
        .map(|s| s.t_ns)
        .min()
        .unwrap();
    for secs in [5, 10, 15, 20, 25, 30, 40, 60] {
        let cut = t0 + secs * 1_000_000_000;
        let mut r = rec.clone();
        for v in r.controllers.values_mut().chain(r.trackers.values_mut()) {
            v.retain(|s| s.t_ns <= cut);
        }
        let t = Instant::now();
        let dets = detect(&r);
        let Some(best) = dets.first() else {
            eprintln!("{secs:3} s: no detection");
            continue;
        };
        let prog = progress(&r, best);
        let dt = t.elapsed();
        let sol = if prog.ready {
            solve(&r, best).ok()
        } else {
            None
        };
        eprintln!(
            "{secs:3} s: {} x {} corr {:.3} (2nd {:.3}) | held {:.1} s diversity {:.2} ready {} | {dt:.1?}{}",
            best.controller,
            best.tracker,
            best.correlation,
            dets.get(1).map_or(f64::NAN, |d| d.correlation),
            prog.held_seconds,
            prog.rotation_diversity,
            prog.ready,
            sol.map_or(String::new(), |s| format!(
                " | solve rms {:.1} mm rot {:.2} deg yaw {:.2} t {:.3?}",
                s.rms_mm,
                s.rot_median_deg,
                yaw_deg(&s.rotation),
                s.translation
            )),
        );
    }
}
