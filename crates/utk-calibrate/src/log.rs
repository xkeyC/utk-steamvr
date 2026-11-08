//! Log file for diagnosing the tool when SteamVR starts it (its stdout/stderr go nowhere useful):
//! `$XDG_STATE_HOME/utk-steamvr/calibrate.log`, truncated on every start.

use std::io::Write;
use std::sync::Mutex;

static FILE: Mutex<Option<std::fs::File>> = Mutex::new(None);

pub fn init() {
    let dir = std::env::var_os("XDG_STATE_HOME")
        .map(std::path::PathBuf::from)
        .or_else(|| {
            std::env::var_os("HOME").map(|h| std::path::PathBuf::from(h).join(".local/state"))
        })
        .map(|d| d.join("utk-steamvr"));
    if let Some(dir) = dir {
        let _ = std::fs::create_dir_all(&dir);
        *FILE.lock().unwrap() = std::fs::File::create(dir.join("calibrate.log")).ok();
    }
}

pub fn write(msg: std::fmt::Arguments) {
    let t = nix::time::clock_gettime(nix::time::ClockId::CLOCK_MONOTONIC)
        .map_or(0.0, |t| t.tv_sec() as f64 + t.tv_nsec() as f64 / 1e9);
    eprintln!("{msg}");
    if let Some(f) = FILE.lock().unwrap().as_mut() {
        let _ = writeln!(f, "{t:.3} {msg}");
    }
}

#[macro_export]
macro_rules! log {
    ($($arg:tt)*) => { $crate::log::write(format_args!($($arg)*)) };
}
