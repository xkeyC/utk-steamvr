//! `utk`: terminal companion for the utk SteamVR driver.
//!
//! - `utk status [--watch]`: the driver's status file (tracker service state, per-tracker sync
//!   state, battery, rate, latency) without putting the headset on.
//! - `utk record`: SteamVR device poses (controllers, HMD, the utk trackers) as JSON Lines, i.e.
//!   exactly what games receive; useful to analyse jitter and latency or to replay calibrations.
//!
//! Recording lines look like
//!   {"src":"steamvr","t":...,"idx":3,"class":"tracker","serial":"...","valid":true,"result":200,"p":[..],"q":[x,y,z,w],"v":[..],"w":[..]}
//! with `t` = CLOCK_MONOTONIC ns. When Steam runs in another network namespace (UU accelerator),
//! run `utk record` through `steamvr-netns`.

mod steamvr;

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, mpsc};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use serde::Deserialize;

#[derive(Parser)]
#[command(
    version,
    about = "VIVE Ultimate Tracker for Linux SteamVR: status and recording"
)]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Show the driver's tracker status
    Status {
        /// Refresh twice a second until Ctrl+C
        #[arg(short, long)]
        watch: bool,
    },
    /// Record SteamVR device poses to a JSON Lines file
    Record {
        /// Output file (default: captures/<unix time>.jsonl)
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// Duration in seconds; records until Ctrl+C if omitted
        #[arg(short, long)]
        duration: Option<f64>,
        /// Sampling rate
        #[arg(long, default_value_t = 90.0)]
        hz: f64,
    },
}

fn main() -> Result<()> {
    match Cli::parse().cmd {
        Cmd::Status { watch } => status(watch),
        Cmd::Record {
            output,
            duration,
            hz,
        } => record(output, duration, hz),
    }
}

pub fn monotonic_ns() -> u64 {
    let t = nix::time::clock_gettime(nix::time::ClockId::CLOCK_MONOTONIC).expect("CLOCK_MONOTONIC");
    t.tv_sec() as u64 * 1_000_000_000 + t.tv_nsec() as u64
}

#[derive(Deserialize)]
struct Status {
    updated_unix_ms: u64,
    server: String,
    #[serde(default)]
    calibration_loaded: bool,
    #[serde(default)]
    trackers: Vec<Tracker>,
}

#[derive(Deserialize)]
struct Tracker {
    serial: String,
    present: bool,
    state: String,
    host: bool,
    rotation_only: bool,
    setup_required: bool,
    stable: bool,
    battery: u8,
    rate_hz: f64,
    latency_ms: f64,
}

fn status_path() -> PathBuf {
    std::env::var_os("XDG_STATE_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".local/state")))
        .unwrap_or_default()
        .join("utk-steamvr/status.json")
}

fn print_status() -> Result<()> {
    let path = status_path();
    let text = match std::fs::read_to_string(&path) {
        Ok(t) => t,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            println!(
                "No status yet: the utk driver has not run. Start SteamVR with the utk add-on enabled."
            );
            return Ok(());
        }
        Err(e) => return Err(e).with_context(|| format!("reading {}", path.display())),
    };
    let s: Status = serde_json::from_str(&text)?;
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)?
        .as_millis() as u64;
    let age = now.saturating_sub(s.updated_unix_ms) as f64 / 1000.0;
    if age > 3.0 {
        println!(
            "The driver is not running (status is {age:.0} s old). Start SteamVR with the utk add-on enabled."
        );
    }
    let server = match s.server.as_str() {
        "running" => "running",
        "vive_hub_running" => "waiting: VIVE Hub is running and holds the dongle, close it",
        "start_failed" => "could not start (is the dongle plugged in?)",
        "starting" => "starting",
        "stopped" => "stopped",
        other => other,
    };
    println!(
        "tracker service: {server}; calibration {}",
        if s.calibration_loaded {
            "loaded"
        } else {
            "missing (identity)"
        }
    );
    println!();
    println!(" serial              state          battery   rate  latency  notes");
    for t in &s.trackers {
        let mut notes = Vec::new();
        if t.host {
            notes.push("host");
        }
        if t.rotation_only {
            notes.push("rotation only");
        }
        if t.setup_required {
            notes.push("needs a map (VIVE Hub)");
        }
        if t.stable {
            notes.push("still");
        }
        if !t.present {
            notes.push("not connected");
        }
        println!(
            " {:<19} {:<14} {:>6}% {:>5.0} Hz {:>5.1} ms  {}",
            t.serial,
            t.state,
            t.battery,
            t.rate_hz,
            t.latency_ms,
            notes.join(", ")
        );
    }
    Ok(())
}

fn status(watch: bool) -> Result<()> {
    if !watch {
        return print_status();
    }
    loop {
        print!("\x1b[H\x1b[2J");
        if let Err(e) = print_status() {
            println!("{e:#}");
        }
        std::io::stdout().flush()?;
        std::thread::sleep(Duration::from_millis(500));
    }
}

fn record(output: Option<PathBuf>, duration: Option<f64>, hz: f64) -> Result<()> {
    let output = output.unwrap_or_else(|| {
        let secs = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs();
        PathBuf::from(format!("captures/{secs}.jsonl"))
    });
    if let Some(dir) = output.parent().filter(|d| !d.as_os_str().is_empty()) {
        std::fs::create_dir_all(dir)?;
    }
    let stop = Arc::new(AtomicBool::new(false));
    {
        let stop = stop.clone();
        ctrlc_handler(move || stop.store(true, Ordering::SeqCst))?;
    }
    let (tx, rx) = mpsc::channel::<String>();
    let file = File::create(&output).with_context(|| format!("creating {}", output.display()))?;
    let writer = std::thread::spawn(move || -> std::io::Result<u64> {
        let mut w = BufWriter::new(file);
        let mut n = 0;
        for line in rx {
            w.write_all(line.as_bytes())?;
            w.write_all(b"\n")?;
            n += 1;
        }
        w.flush()?;
        Ok(n)
    });
    let sampler = {
        let stop = stop.clone();
        std::thread::spawn(move || steamvr::sample(tx, stop, hz))
    };
    eprintln!("Recording to {} (Ctrl+C to stop)", output.display());
    let started = Instant::now();
    while !stop.load(Ordering::SeqCst)
        && !sampler.is_finished()
        && duration.is_none_or(|d| started.elapsed().as_secs_f64() < d)
    {
        std::thread::sleep(Duration::from_millis(50));
    }
    stop.store(true, Ordering::SeqCst);
    sampler.join().expect("sampler thread panicked")?;
    let lines = writer.join().expect("writer thread panicked")?;
    eprintln!(
        "Done: {:.1} s, {lines} lines written",
        started.elapsed().as_secs_f64()
    );
    Ok(())
}

/// SIGINT handling without extra dependencies: the first Ctrl+C only sets a flag so the recording ends cleanly.
fn ctrlc_handler(f: impl Fn() + Send + 'static) -> Result<()> {
    use nix::sys::signal::{SigSet, Signal};
    let mut set = SigSet::empty();
    set.add(Signal::SIGINT);
    set.thread_block()?;
    std::thread::spawn(move || {
        if set.wait().is_ok() {
            f();
        }
    });
    Ok(())
}
