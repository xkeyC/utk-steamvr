//! Reads SteamVR device poses as an OpenVR background app, in the raw (uncalibrated) universe:
//! calibration input must not depend on the play area setup, and the driver reports in raw space too.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, mpsc};
use std::time::{Duration, Instant};

use anyhow::{Result, anyhow};
use ovr_overlay::errors::InitError;
use ovr_overlay::sys::{
    ETrackedControllerRole, ETrackedDeviceClass, ETrackedDeviceProperty, ETrackingUniverseOrigin,
    EVRApplicationType,
};
use ovr_overlay::{Context, TrackedDeviceIndex};
use serde::Serialize;

#[derive(Serialize)]
struct Line<'a> {
    src: &'static str,
    t: u64,
    idx: u32,
    class: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    role: Option<&'static str>,
    serial: &'a str,
    valid: bool,
    /// Raw ETrackingResult (200 = Running_OK), see openvr.h
    result: i32,
    p: [f32; 3],
    q: [f32; 4],
    v: [f32; 3],
    w: [f32; 3],
}

pub fn sample(tx: mpsc::Sender<String>, stop: Arc<AtomicBool>, hz: f64) -> Result<()> {
    let ctx = Context::init(EVRApplicationType::VRApplication_Background).map_err(|e| match e {
        InitError::Sys(e) => anyhow!(
            "cannot connect to SteamVR (is it running, and in this network namespace?): {e}"
        ),
        InitError::AlreadyInitialized => anyhow!("OpenVR is already initialized"),
    })?;
    let mut sys = ctx.system_mngr();
    let mut serials: Vec<Option<String>> =
        vec![None; ovr_overlay::sys::k_unMaxTrackedDeviceCount as usize];
    let period = Duration::from_secs_f64(1.0 / hz);
    let mut next = Instant::now();
    eprintln!("SteamVR: connected, sampling the raw universe at {hz} Hz");
    while !stop.load(Ordering::SeqCst) {
        let poses = sys.get_device_to_absolute_tracking_pose(
            ETrackingUniverseOrigin::TrackingUniverseRawAndUncalibrated,
            0.0,
        );
        let t = crate::monotonic_ns();
        for (i, pose) in poses.iter().enumerate() {
            if !pose.bDeviceIsConnected {
                continue;
            }
            let index = TrackedDeviceIndex(i as u32);
            let class = match sys.get_tracked_device_class(index) {
                ETrackedDeviceClass::TrackedDeviceClass_HMD => "hmd",
                ETrackedDeviceClass::TrackedDeviceClass_Controller => "controller",
                ETrackedDeviceClass::TrackedDeviceClass_GenericTracker => "tracker",
                _ => continue,
            };
            let role = (class == "controller").then(|| {
                match sys.get_controller_role_for_tracked_device_index(index) {
                    ETrackedControllerRole::TrackedControllerRole_LeftHand => "left",
                    ETrackedControllerRole::TrackedControllerRole_RightHand => "right",
                    _ => "other",
                }
            });
            let serial = serials[i].get_or_insert_with(|| {
                sys.get_tracked_device_property::<String>(
                    index,
                    ETrackedDeviceProperty::Prop_SerialNumber_String,
                )
                .unwrap_or_default()
            });
            let m = pose.mDeviceToAbsoluteTracking.m;
            let line = Line {
                src: "steamvr",
                t,
                idx: i as u32,
                class,
                role,
                serial,
                valid: pose.bPoseIsValid,
                result: pose.eTrackingResult.clone() as i32,
                p: [m[0][3], m[1][3], m[2][3]],
                q: matrix_to_quat(&m),
                v: pose.vVelocity.v,
                w: pose.vAngularVelocity.v,
            };
            if tx.send(serde_json::to_string(&line)?).is_err() {
                return Ok(());
            }
        }
        // Forget cached serials of disconnected devices so a replacement device is read again.
        for (i, pose) in poses.iter().enumerate() {
            if !pose.bDeviceIsConnected {
                serials[i] = None;
            }
        }
        next += period;
        let now = Instant::now();
        if next > now {
            std::thread::sleep(next - now);
        } else {
            next = now;
        }
    }
    Ok(())
}

/// Rotation part of a 3x4 pose matrix as a quaternion (x, y, z, w).
fn matrix_to_quat(m: &[[f32; 4]; 3]) -> [f32; 4] {
    let (m00, m01, m02) = (m[0][0], m[0][1], m[0][2]);
    let (m10, m11, m12) = (m[1][0], m[1][1], m[1][2]);
    let (m20, m21, m22) = (m[2][0], m[2][1], m[2][2]);
    let trace = m00 + m11 + m22;
    let q = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        [(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s]
    } else if m00 > m11 && m00 > m22 {
        let s = (1.0 + m00 - m11 - m22).sqrt() * 2.0;
        [0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s]
    } else if m11 > m22 {
        let s = (1.0 + m11 - m00 - m22).sqrt() * 2.0;
        [(m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s]
    } else {
        let s = (1.0 + m22 - m00 - m11).sqrt() * 2.0;
        [(m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s]
    };
    let n = q.iter().map(|v| v * v).sum::<f32>().sqrt();
    q.map(|v| v / n)
}
