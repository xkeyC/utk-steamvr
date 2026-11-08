//! Controller buttons through SteamVR Input: A starts a calibration, B quits, Y shows the current
//! calibration. Overlay panels cannot easily receive laser-pointer clicks, buttons can.

use std::path::PathBuf;

use anyhow::{Result, anyhow};
use ovr_overlay::input::{
    ActionHandle, ActionSetHandle, ActiveActionSet, InputManager, InputValueHandle,
};
use ovr_overlay::sys;

const ACTIONS: &str = include_str!("../input/actions.json");
const BINDINGS_OCULUS_TOUCH: &str = include_str!("../input/bindings_oculus_touch.json");

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Pressed {
    pub start: bool,
    pub quit: bool,
    pub show: bool,
    /// Whether SteamVR currently delivers each action (start, quit, show) to this app; false
    /// while another app or the SteamVR menu has taken the input.
    pub active: [bool; 3],
}

/// Writes the action manifest and default bindings under $HOME and returns the manifest path.
/// vrserver reads these files itself and runs in pressure-vessel, which only sees $HOME, so they
/// cannot be read from the build tree.
pub fn write_manifest() -> Result<PathBuf> {
    let dir = std::env::var_os("XDG_DATA_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".local/share")))
        .ok_or_else(|| anyhow!("no HOME"))?
        .join("utk-steamvr/input");
    std::fs::create_dir_all(&dir)?;
    std::fs::write(dir.join("actions.json"), ACTIONS)?;
    std::fs::write(
        dir.join("bindings_oculus_touch.json"),
        BINDINGS_OCULUS_TOUCH,
    )?;
    Ok(dir.join("actions.json"))
}

pub struct Buttons {
    set: ActionSetHandle,
    start: ActionHandle,
    quit: ActionHandle,
    show: ActionHandle,
}

impl Buttons {
    pub fn new(input: &mut InputManager) -> Result<Self> {
        let manifest = write_manifest()?;
        input
            .set_action_manifest(&manifest)
            .map_err(|e| anyhow!("SetActionManifestPath: {e}"))?;
        let get = |input: &mut InputManager, name: &str| {
            input
                .get_action_handle(name)
                .map_err(|e| anyhow!("action {name}: {e}"))
        };
        Ok(Self {
            set: input
                .get_action_set_handle("/actions/utk")
                .map_err(|e| anyhow!("action set: {e}"))?,
            start: get(input, "/actions/utk/in/start")?,
            quit: get(input, "/actions/utk/in/quit")?,
            show: get(input, "/actions/utk/in/show")?,
        })
    }

    /// Buttons that went down since the last call.
    pub fn poll(&self, input: &mut InputManager) -> Pressed {
        let mut sets = [ActiveActionSet(sys::VRActiveActionSet_t {
            ulActionSet: self.set.0,
            ulRestrictedToDevice: sys::k_ulInvalidInputValueHandle,
            ulSecondaryActionSet: 0,
            unPadding: 0,
            nPriority: 0,
        })];
        if input.update_actions(&mut sets).is_err() {
            return Pressed::default();
        }
        let read = |input: &mut InputManager, a: &ActionHandle| {
            input
                .get_digital_action_data(*a, InputValueHandle(sys::k_ulInvalidInputValueHandle))
                .map_or((false, false), |d| {
                    (d.0.bActive, d.0.bActive && d.0.bState && d.0.bChanged)
                })
        };
        let (start, quit, show) = (
            read(input, &self.start),
            read(input, &self.quit),
            read(input, &self.show),
        );
        Pressed {
            start: start.1,
            quit: quit.1,
            show: show.1,
            active: [start.0, quit.0, show.0],
        }
    }
}
