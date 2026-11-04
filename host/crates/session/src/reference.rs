//! A session and how to configure the core for it, as the vectors are: accelerometer fit
//! from a tumble session, initial gyro bias from a rest session, still thresholds.

use std::fs;
use std::path::{Path, PathBuf};

use anyhow::Context;
use clap::{Args, ValueEnum};

use crate::recording::{self, Recording};
use crate::{calib, config::Config};

/// A session and how to configure the core for it (`path`, `events`, `strokes`).
#[derive(Debug, Args)]
pub struct ReferenceArgs {
    /// Session file (`.bin`).
    pub file: PathBuf,
    /// Tumble session to calibrate the accelerometer with (`calib`).
    #[arg(long)]
    pub calib: Option<PathBuf>,
    /// Still thresholds: on a table or in the hand.
    #[arg(long, value_enum, default_value_t = Hold::Hand)]
    pub hold: Hold,
    /// Rest session (`.bin`) giving the initial gyro bias [default: this session's
    /// median].
    #[arg(long)]
    pub bias: Option<PathBuf>,
}

/// Where the board rests between strokes.
#[derive(Clone, Copy, Debug, ValueEnum)]
pub enum Hold {
    Hand,
    Table,
}

/// Reads and decodes a session file.
pub fn load(p: &Path) -> anyhow::Result<Recording> {
    let bytes = fs::read(p).with_context(|| format!("reading {}", p.display()))?;
    recording::load(&bytes)
}

/// The session, its config, and a line saying where the config came from.
pub fn reference_config(
    ReferenceArgs {
        file,
        calib,
        hold,
        bias,
    }: &ReferenceArgs,
) -> anyhow::Result<(Recording, Config, String)> {
    let fit = calib
        .as_deref()
        .map(|c| calib::fit_session(&load(c)?))
        .transpose()?;
    let rec = load(file)?;
    // The initial gyro bias, as a caller would measure it: a rest session's mean,
    // else this session's median (mostly still).
    let bias_dps = match bias {
        Some(b) => recording::mean(&recording::scaled(&load(b)?)?.gyro),
        None => recording::median(&recording::scaled(&rec)?.gyro),
    };
    let still = match hold {
        Hold::Hand => calib::HAND,
        Hold::Table => calib::TABLE,
    };
    let cfg = Config::new(&rec, fit.as_ref(), bias_dps, still)?;
    let about = format!(
        "accel {}, gyro bias {:.2} {:.2} {:.2} dps ({}), {hold:?} thresholds\n",
        if fit.is_some() { "calibrated" } else { "raw" },
        bias_dps[0],
        bias_dps[1],
        bias_dps[2],
        if bias.is_some() {
            "rest session"
        } else {
            "this session's median"
        }
    );
    Ok((rec, cfg, about))
}
