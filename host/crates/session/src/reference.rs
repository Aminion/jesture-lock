//! A session and how to configure the core for it, as the vectors are: accelerometer fit
//! from a tumble session, initial gyro bias from a rest session, still thresholds.

use std::fs;
use std::path::{Path, PathBuf};

use anyhow::Context;
use clap::{Args, ValueEnum};
use protocol::Hello;

use crate::calib::{self, Fit, Still};
use crate::config::Config;
use crate::recording::{self, Recording, V3};

/// How to configure the core, for any stream (`path`, `events`, `strokes`, the viewer).
#[derive(Debug, Clone, Args)]
pub struct CoreArgs {
    /// Tumble session to calibrate the accelerometer with (`calib`).
    #[arg(long)]
    pub calib: Option<PathBuf>,
    /// Still thresholds: on a table or in the hand.
    #[arg(long, value_enum, default_value_t = Hold::Hand)]
    pub hold: Hold,
    /// Rest session (`.bin`) giving the initial gyro bias [default: this session's
    /// median; zero on a live stream].
    #[arg(long)]
    pub bias: Option<PathBuf>,
}

/// The calibration and rest sessions the vectors are configured with
/// (`core/contract/vectors/v1/README.md`), in `data/sessions/`.
pub const VECTOR_TUMBLE: &str = "20261030-205757_tumble.bin";
pub const VECTOR_REST: &str = "20261030-183544_rest-table.bin";

impl CoreArgs {
    /// As the vectors are configured: `VECTOR_TUMBLE` and `VECTOR_REST` from `sessions`,
    /// hand thresholds.
    pub fn vectors(sessions: &Path) -> Self {
        Self {
            calib: Some(sessions.join(VECTOR_TUMBLE)),
            hold: Hold::Hand,
            bias: Some(sessions.join(VECTOR_REST)),
        }
    }
}

/// A session and how to configure the core for it (`path`, `events`, `strokes`).
#[derive(Debug, Clone, Args)]
pub struct ReferenceArgs {
    /// Session file (`.bin`).
    pub file: PathBuf,
    #[command(flatten)]
    pub core: CoreArgs,
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

/// `CoreArgs` with its files read: what is known before the stream's `Hello`.
#[derive(Debug, Clone, Copy)]
pub struct Setup {
    pub fit: Option<Fit>,
    /// From the rest session, if one was given.
    pub bias_dps: Option<V3>,
    pub hold: Hold,
}

impl Setup {
    pub fn load(CoreArgs { calib, hold, bias }: &CoreArgs) -> anyhow::Result<Self> {
        let fit = calib
            .as_deref()
            .map(|c| calib::fit_session(&load(c)?))
            .transpose()?;
        let bias_dps = bias
            .as_deref()
            .map(|b| anyhow::Ok(recording::mean(&recording::scaled(&load(b)?)?.gyro)))
            .transpose()?;
        Ok(Self {
            fit,
            bias_dps,
            hold: *hold,
        })
    }

    /// The config for a stream that starts with `hello`, and a line saying where it came
    /// from. `fallback_bias` gives the initial bias (and its source) when no rest session
    /// was given.
    pub fn config(
        &self,
        hello: &Hello,
        fallback_bias: impl FnOnce() -> anyhow::Result<(V3, &'static str)>,
    ) -> anyhow::Result<(Config, String)> {
        let (bias_dps, bias_from) = match self.bias_dps {
            Some(b) => (b, "rest session"),
            None => fallback_bias()?,
        };
        let still: Still = match self.hold {
            Hold::Hand => calib::HAND,
            Hold::Table => calib::TABLE,
        };
        let cfg = Config::new(hello, self.fit.as_ref(), bias_dps, still)?;
        let about = format!(
            "accel {}, gyro bias {:.2} {:.2} {:.2} dps ({bias_from}), {:?} thresholds\n",
            if self.fit.is_some() {
                "calibrated"
            } else {
                "raw"
            },
            bias_dps[0],
            bias_dps[1],
            bias_dps[2],
            self.hold
        );
        Ok((cfg, about))
    }
}

/// The session, its config, and a line saying where the config came from.
pub fn reference_config(
    ReferenceArgs { file, core }: &ReferenceArgs,
) -> anyhow::Result<(Recording, Config, String)> {
    let setup = Setup::load(core)?;
    let rec = load(file)?;
    // The initial gyro bias, as a caller would measure it: a rest session's mean,
    // else this session's median (mostly still).
    let (cfg, about) = setup.config(&rec.hello, || {
        let median = recording::median(&recording::scaled(&rec)?.gyro);
        Ok((median, "this session's median"))
    })?;
    Ok((rec, cfg, about))
}
