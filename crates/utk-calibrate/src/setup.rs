//! Offline installation: registers the driver and the calibration app with SteamVR by editing its
//! configuration files, so nothing has to talk to a running SteamVR.
//!
//! - `~/.config/openvr/openvrpaths.vrpath`: the driver directory in `external_drivers`.
//! - `<Steam>/config/steamvr.vrsettings`: `steamvr/activateMultipleDrivers = true`. With it off,
//!   SteamVR loads only the driver that provides the HMD (e.g. Steam Link's vrlink), never ours.
//! - `<Steam>/config/appconfig.json`: our `.vrmanifest` in `manifest_paths`, which makes the
//!   calibration a tab in the SteamVR menu that SteamVR starts on its own.
//!
//! vrserver rewrites these files when it exits, so they are only edited while it is not running.
//! Every file is backed up once as `*.utk-backup` before its first change.

use std::os::unix::fs::MetadataExt;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, anyhow, bail};
use serde_json::{Map, Value, json};

fn config_home() -> Result<PathBuf> {
    std::env::var_os("XDG_CONFIG_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))
        .ok_or_else(|| anyhow!("no HOME"))
}

fn vrpath_file() -> Result<PathBuf> {
    Ok(config_home()?.join("openvr/openvrpaths.vrpath"))
}

/// True when a vrserver of this user is running.
fn steamvr_running() -> bool {
    let Ok(me) = std::fs::metadata("/proc/self").map(|m| m.uid()) else {
        return false;
    };
    let Ok(dir) = std::fs::read_dir("/proc") else {
        return false;
    };
    dir.flatten().any(|e| {
        e.metadata().is_ok_and(|m| m.uid() == me)
            && std::fs::read_to_string(e.path().join("comm")).is_ok_and(|c| c.trim() == "vrserver")
    })
}

fn read_json(path: &Path) -> Result<Value> {
    match std::fs::read_to_string(path) {
        Ok(text) => {
            serde_json::from_str(&text).with_context(|| format!("parsing {}", path.display()))
        }
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(Value::Object(Map::new())),
        Err(e) => Err(e).with_context(|| format!("reading {}", path.display())),
    }
}

/// A JSON file to update; `indent` follows what SteamVR itself writes.
struct Edit {
    path: PathBuf,
    indent: &'static [u8],
    before: Value,
    after: Value,
}

impl Edit {
    fn open(path: PathBuf, indent: &'static [u8]) -> Result<Self> {
        let before = read_json(&path)?;
        if !before.is_object() {
            bail!("{} is not a JSON object", path.display());
        }
        Ok(Self {
            path,
            indent,
            after: before.clone(),
            before,
        })
    }

    fn obj(&mut self) -> &mut Map<String, Value> {
        self.after.as_object_mut().expect("checked in open")
    }

    /// Adds or removes `item` in the string array `key` (null when it ends up empty, as SteamVR
    /// writes `external_drivers`).
    fn set_member(&mut self, key: &str, item: &str, present: bool, null_if_empty: bool) {
        let obj = self.obj();
        let mut list: Vec<Value> = obj
            .get(key)
            .and_then(Value::as_array)
            .cloned()
            .unwrap_or_default();
        let had = list.iter().any(|v| v.as_str() == Some(item));
        if present && had {
            return;
        }
        list.retain(|v| v.as_str() != Some(item));
        if present {
            list.push(Value::String(item.to_owned()));
        }
        let value = if list.is_empty() && null_if_empty {
            Value::Null
        } else {
            Value::Array(list)
        };
        obj.insert(key.to_owned(), value);
    }

    fn changed(&self) -> bool {
        self.before != self.after
    }

    fn save(&self) -> Result<()> {
        if !self.changed() {
            return Ok(());
        }
        let backup = PathBuf::from(format!("{}.utk-backup", self.path.display()));
        if self.path.exists() && !backup.exists() {
            std::fs::copy(&self.path, &backup)
                .with_context(|| format!("backing up {}", self.path.display()))?;
        }
        if let Some(dir) = self.path.parent() {
            std::fs::create_dir_all(dir)?;
        }
        let mut out = Vec::new();
        let fmt = serde_json::ser::PrettyFormatter::with_indent(self.indent);
        let mut ser = serde_json::Serializer::with_formatter(&mut out, fmt);
        serde::Serialize::serialize(&self.after, &mut ser)?;
        out.push(b'\n');
        let tmp = self.path.with_extension("utk-tmp");
        std::fs::write(&tmp, out)?;
        std::fs::rename(&tmp, &self.path)
            .with_context(|| format!("writing {}", self.path.display()))?;
        println!("updated {}", self.path.display());
        Ok(())
    }
}

/// The SteamVR config files to edit for (un)installing, with the changes applied in memory.
fn plan(driver_dir: &Path, manifest: &Path, install: bool) -> Result<Vec<Edit>> {
    let vrpath = vrpath_file()?;
    if !vrpath.exists() {
        bail!(
            "{} does not exist: install SteamVR and start it once first",
            vrpath.display()
        );
    }
    let mut paths = Edit::open(vrpath, b"\t")?;
    let steam_config = paths
        .before
        .get("config")
        .and_then(Value::as_array)
        .and_then(|a| a.first())
        .and_then(Value::as_str)
        .map(PathBuf::from)
        .ok_or_else(|| anyhow!("no \"config\" directory in {}", paths.path.display()))?;
    paths.set_member(
        "external_drivers",
        &driver_dir.to_string_lossy(),
        install,
        true,
    );

    let mut apps = Edit::open(steam_config.join("appconfig.json"), b"   ")?;
    apps.set_member(
        "manifest_paths",
        &manifest.to_string_lossy(),
        install,
        false,
    );

    let mut edits = vec![paths, apps];
    if install {
        // Left on when uninstalling: other add-on drivers may rely on it too.
        let mut settings = Edit::open(steam_config.join("steamvr.vrsettings"), b"   ")?;
        let steamvr = settings.obj().entry("steamvr").or_insert_with(|| json!({}));
        let Some(steamvr) = steamvr.as_object_mut() else {
            bail!("steamvr in {} is not an object", settings.path.display());
        };
        steamvr.insert("activateMultipleDrivers".into(), Value::Bool(true));
        edits.push(settings);
    }
    Ok(edits)
}

fn apply(edits: &[Edit]) -> Result<()> {
    if edits.iter().any(Edit::changed) && steamvr_running() {
        bail!(
            "SteamVR is running and would overwrite its settings when it exits: quit SteamVR and run this again"
        );
    }
    edits.iter().try_for_each(Edit::save)
}

/// The installation root: this binary lives in `<root>/bin`.
fn install_root() -> Result<PathBuf> {
    let exe = std::env::current_exe()?.canonicalize()?;
    exe.parent()
        .and_then(Path::parent)
        .map(Path::to_path_buf)
        .ok_or_else(|| anyhow!("unexpected location {}", exe.display()))
}

/// `--install`: registers `<root>/driver/utk` and this binary with SteamVR.
pub fn install() -> Result<()> {
    let root = install_root()?;
    let driver_dir = root.join("driver/utk");
    for f in [
        "driver.vrdrivermanifest",
        "bin/linux64/driver_utk.so",
        "bin/linux64/libultimate-tracker.so",
    ] {
        if !driver_dir.join(f).exists() {
            bail!("missing {}", driver_dir.join(f).display());
        }
    }
    let exe = std::env::current_exe()?.canonicalize()?;
    let actions = crate::input::write_manifest()?;
    // The dashboard app renders the icon there whenever it starts.
    let icon = crate::data_dir().join("icon.png");
    let manifest = crate::dashboard::write_vrmanifest(&exe, &actions, Some(&icon))?;
    apply(&plan(&driver_dir, &manifest, true)?)?;
    println!("SteamVR driver: {}", driver_dir.display());
    println!("SteamVR menu app: {}", manifest.display());
    Ok(())
}

/// `--uninstall`: removes both registrations (the files themselves stay).
pub fn uninstall() -> Result<()> {
    let root = install_root()?;
    let manifest = std::env::current_exe()?
        .canonicalize()?
        .with_file_name("utk-calibrate.vrmanifest");
    apply(&plan(&root.join("driver/utk"), &manifest, false)?)?;
    println!("unregistered the utk driver and the SteamVR menu app");
    Ok(())
}
