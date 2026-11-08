//! SteamVR dashboard overlay (a tab in the SteamVR menu with a thumbnail icon) driven through the
//! raw IVROverlay interface; ovr_overlay does not wrap these calls. The laser pointer arrives as
//! mouse events, which are fed to egui.

use std::ffi::CString;
use std::pin::Pin;

use anyhow::{Result, anyhow};
use ovr_overlay::sys;

const EVENT_MOUSE_MOVE: u32 = 300;
const EVENT_MOUSE_DOWN: u32 = 301;
const EVENT_MOUSE_UP: u32 = 302;

fn ivr_overlay() -> Pin<&'static mut sys::IVROverlay> {
    // SAFETY: VROverlay() is valid once OpenVR is initialized, which main() guarantees.
    unsafe { Pin::new_unchecked(sys::VROverlay().as_mut().expect("IVROverlay")) }
}

pub struct Dashboard {
    pub main: u64,
    pub thumbnail: u64,
    size: (f32, f32),
    pixels_per_point: f32,
}

impl Dashboard {
    pub fn new(key: &str, name: &str, size_px: (u32, u32), pixels_per_point: f32) -> Result<Self> {
        let (key, name) = (CString::new(key)?, CString::new(name)?);
        let (mut main, mut thumbnail) = (0u64, 0u64);
        let err = unsafe {
            ivr_overlay().CreateDashboardOverlay(
                key.as_ptr(),
                name.as_ptr(),
                &mut main,
                &mut thumbnail,
            )
        };
        if err != sys::EVROverlayError::VROverlayError_None {
            return Err(anyhow!("CreateDashboardOverlay failed: {}", err as i32));
        }
        let size = (size_px.0 as f32, size_px.1 as f32);
        unsafe {
            ivr_overlay()
                .SetOverlayInputMethod(main, sys::VROverlayInputMethod::VROverlayInputMethod_Mouse);
            ivr_overlay().SetOverlayMouseScale(
                main,
                &sys::HmdVector2_t {
                    v: [size.0, size.1],
                },
            );
        }
        Ok(Self {
            main,
            thumbnail,
            size,
            pixels_per_point,
        })
    }

    /// Menu-bar icon from a PNG file (SteamVR loads it itself, so it must be under $HOME).
    pub fn set_thumbnail_file(&self, path: &std::path::Path) -> Result<()> {
        let c = CString::new(path.to_string_lossy().as_bytes())?;
        let err = unsafe { ivr_overlay().SetOverlayFromFile(self.thumbnail, c.as_ptr()) };
        if err != sys::EVROverlayError::VROverlayError_None {
            return Err(anyhow!("SetOverlayFromFile failed: {}", err as i32));
        }
        Ok(())
    }

    pub fn is_active(&self) -> bool {
        unsafe { ivr_overlay().IsActiveDashboardOverlay(self.main) }
    }

    /// Laser-pointer events since the last call, as egui input (points, y down).
    pub fn poll(&self) -> Vec<egui::Event> {
        let mut out = Vec::new();
        loop {
            let mut ev = std::mem::MaybeUninit::<sys::VREvent_t>::zeroed();
            let got = unsafe {
                ivr_overlay().PollNextOverlayEvent(
                    self.main,
                    ev.as_mut_ptr(),
                    std::mem::size_of::<sys::VREvent_t>() as u32,
                )
            };
            if !got {
                return out;
            }
            // VREvent_t is packed to 4 bytes on Linux: eventType, trackedDeviceIndex,
            // eventAgeSeconds, then the data union; VREvent_Mouse_t is { float x, y; uint32 button }.
            let bytes = unsafe {
                std::slice::from_raw_parts(
                    ev.as_ptr() as *const u8,
                    std::mem::size_of::<sys::VREvent_t>(),
                )
            };
            let u32_at = |o: usize| u32::from_le_bytes(bytes[o..o + 4].try_into().unwrap());
            let f32_at = |o: usize| f32::from_le_bytes(bytes[o..o + 4].try_into().unwrap());
            let kind = u32_at(0);
            // Mouse y grows upwards in OpenVR.
            let pos = egui::pos2(f32_at(12), self.size.1 - f32_at(16)) / self.pixels_per_point;
            match kind {
                EVENT_MOUSE_MOVE => out.push(egui::Event::PointerMoved(pos)),
                EVENT_MOUSE_DOWN | EVENT_MOUSE_UP => {
                    out.push(egui::Event::PointerMoved(pos));
                    out.push(egui::Event::PointerButton {
                        pos,
                        button: egui::PointerButton::Primary,
                        pressed: kind == EVENT_MOUSE_DOWN,
                        modifiers: egui::Modifiers::default(),
                    });
                }
                _ => {}
            }
        }
    }
}

/// Writes `utk-calibrate.vrmanifest` next to `binary`: a SteamVR dashboard-overlay application
/// that SteamVR starts on its own (`--dashboard`), so the calibration lives in the SteamVR menu.
pub fn write_vrmanifest(
    binary: &std::path::Path,
    actions: &std::path::Path,
    icon: Option<&std::path::Path>,
) -> Result<std::path::PathBuf> {
    let manifest = binary.with_file_name("utk-calibrate.vrmanifest");
    let json = serde_json::json!({
        "source": "builtin",
        "applications": [{
            "app_key": crate::APP_KEY,
            "launch_type": "binary",
            "binary_path_linux": binary,
            "arguments": "--dashboard",
            "is_dashboard_overlay": true,
            "action_manifest_path": actions,
            "image_path": icon,
            "strings": { "en_us": { "name": "UTK Calibration", "description": "Calibrate VIVE Ultimate Trackers" } }
        }]
    });
    std::fs::write(&manifest, serde_json::to_string_pretty(&json)? + "\n")?;
    Ok(manifest)
}

/// Registers the manifest with a running SteamVR (`--register`).
pub fn register(
    apps: &mut ovr_overlay::applications::ApplicationsManager,
    binary: &std::path::Path,
    actions: &std::path::Path,
    icon: Option<&std::path::Path>,
) -> Result<std::path::PathBuf> {
    let manifest = write_vrmanifest(binary, actions, icon)?;
    apps.add_application_manifest(&manifest, false)
        .map_err(|e| anyhow!("AddApplicationManifest: {e}"))?;
    apps.set_application_auto_launch(crate::APP_KEY, true)
        .map_err(|e| anyhow!("SetApplicationAutoLaunch: {e}"))?;
    Ok(manifest)
}
