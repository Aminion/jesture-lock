//! A session and how to configure the core for it, as the vectors are: accelerometer fit
//! from a tumble session, initial gyro bias from a rest session, still thresholds.

use std::fs;
use std::path::{Path, PathBuf};

use anyhow::Context;
use clap::{Args, ValueEnum};
use protocol::Hello;

use crate::calfile::{self, CalFile};
use crate::calib::{self, Fit, Still};
use crate::config::Config;
use crate::recording::{self, Recording, V3};

/// How to configure the core, for any stream (`path`, `events`, `strokes`, the viewer).
#[derive(Debug, Clone, Args)]
pub struct CoreArgs {
    /// Accelerometer calibration: a calibration file (`calibrate`, `.cal`; it also gives
    /// the initial gyro bias) or a tumble session to fit (`calib`).
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
    /// From a tumble session.
    pub fit: Option<Fit>,
    /// A calibration file: accel calibration, and the gyro bias unless a rest session
    /// was given.
    pub file: Option<CalFile>,
    /// The linked `board.cal`, when no `--calib` was given: the default for streams of
    /// its scales (`docs/calibration.md`).
    pub linked: Option<CalFile>,
    /// From the rest session, if one was given.
    pub bias_dps: Option<V3>,
    pub hold: Hold,
}

impl Setup {
    pub fn load(CoreArgs { calib, hold, bias }: &CoreArgs) -> anyhow::Result<Self> {
        let (mut fit, mut file) = (None, None);
        if let Some(c) = calib {
            let bytes = fs::read(c).with_context(|| format!("reading {}", c.display()))?;
            if bytes.starts_with(&calfile::MAGIC) {
                file = Some(CalFile::parse(&bytes).with_context(|| c.display().to_string())?);
            } else {
                fit = Some(calib::fit_session(&recording::load(&bytes)?)?);
            }
        }
        let bias_dps = bias
            .as_deref()
            .map(|b| anyhow::Ok(recording::mean(&recording::scaled(&load(b)?)?.gyro)))
            .transpose()?;
        let linked = match calib {
            Some(_) => None,
            None => Some(calfile::board().context("the linked board.cal")?),
        };
        Ok(Self {
            fit,
            file,
            linked,
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
        // The linked file is the default where its scales fit; elsewhere the default stays
        // raw, as before there was a file.
        let linked = self
            .linked
            .filter(|f| f.applies_to(hello.ctrl1_xl, hello.ctrl2_g));
        let file = self.file.or(linked);
        let (bias_dps, bias_from) = match (self.bias_dps, file) {
            (Some(b), _) => (b, "rest session"),
            // Replaced by the file's below.
            (None, Some(_)) => ([0.0; 3], "calibration file"),
            (None, None) => fallback_bias()?,
        };
        let still: Still = match self.hold {
            Hold::Hand => calib::HAND,
            Hold::Table => calib::TABLE,
        };
        let mut cfg = Config::new(hello, self.fit.as_ref(), bias_dps, still)?;
        let mut bias_dps = bias_dps;
        if let Some(file) = &file {
            let bias_lsb = cfg.bias_lsb;
            cfg = cfg.with_file(file)?;
            if self.bias_dps.is_some() {
                cfg.bias_lsb = bias_lsb;
            } else {
                bias_dps = cfg.bias_dps();
            }
        }
        let about = format!(
            "accel {}, gyro bias {:.2} {:.2} {:.2} dps ({bias_from}), {:?} thresholds\n",
            match (file, self.fit) {
                (Some(_), _) if self.file.is_some() => "calibrated (file)",
                (Some(_), _) => "calibrated (linked board.cal)",
                (None, Some(_)) => "calibrated",
                (None, None) if self.linked.is_some() => {
                    "raw (the linked board.cal is for other scales)"
                }
                (None, None) => "raw",
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

#[cfg(test)]
mod tests {
    use protocol::{Firmware, ImuStatus};

    use super::*;

    const FILE: CalFile = CalFile {
        ctrl1_xl: 0x48,
        ctrl2_g: 0x44,
        accel_offset_lsb: [-35, 45, 205],
        accel_gain_q30: [1_083_727_709, 1_072_668_082, 1_068_051_012],
        gyro_bias_q8: [26_581, -82_990, -3_511],
    };

    fn hello(ctrl1_xl: u8, ctrl2_g: u8) -> Hello {
        Hello {
            fw: Firmware::Stream,
            imu: ImuStatus::Ok,
            imu_addr: 0x6B,
            who_am_i: 0x69,
            ctrl1_xl,
            ctrl2_g,
        }
    }

    /// `--calib <file.cal>`: the file's integers reach `gc_config` unchanged, with its
    /// bias unless a rest session gives one; other scales are an error.
    #[test]
    fn setup_with_a_calibration_file() {
        let dir = std::env::temp_dir().join(format!("gesture-reference-{}", std::process::id()));
        fs::create_dir_all(&dir).expect("dir");
        let path = dir.join("board.cal");
        fs::write(&path, FILE.to_bytes()).expect("write");
        let args = |bias| CoreArgs {
            calib: Some(path.clone()),
            hold: Hold::Hand,
            bias,
        };
        let no_fallback = || -> anyhow::Result<(V3, &'static str)> { panic!("not asked") };

        let setup = Setup::load(&args(None)).expect("setup");
        // Another ODR, the same scales.
        let (cfg, about) = setup
            .config(&hello(0x68, 0x54), no_fallback)
            .expect("config");
        let gc = cfg.gc();
        assert_eq!((gc.ctrl1_xl, gc.ctrl2_g), (0x68, 0x54));
        assert_eq!(
            CalFile::from_gc(&gc),
            CalFile {
                ctrl1_xl: 0x68,
                ctrl2_g: 0x54,
                ..FILE
            }
        );
        assert!(about.contains("calibrated (file)") && about.contains("1.82 -5.67 -0.24"));
        // The thresholds are not the file's business: the hand preset (docs/calibration.md).
        assert_eq!(
            (
                gc.still_gyro_std_lsb,
                gc.still_accel_std_lsb,
                gc.still_gyro_rate_lsb
            ),
            (171, 123, 229)
        );

        let err = setup
            .config(&hello(0x40, 0x40), no_fallback)
            .expect_err("scales");
        assert!(err.to_string().contains("calibration file is for"), "{err}");

        let rest = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../../data/sessions/20261030-213745_rest-table-4g.bin");
        let setup = Setup::load(&args(Some(rest))).expect("setup");
        let (cfg, about) = setup
            .config(&hello(0x48, 0x44), no_fallback)
            .expect("config");
        let gc = cfg.gc();
        assert_eq!(gc.accel_gain_q30, FILE.accel_gain_q30);
        assert_ne!(gc.gyro_bias_q8, FILE.gyro_bias_q8);
        assert!(about.contains("rest session"), "{about}");
        fs::remove_dir_all(&dir).expect("cleanup");
    }

    /// Without `--calib`: the linked `board.cal` where its scales fit, raw elsewhere.
    #[test]
    fn linked_file_is_the_default() {
        let setup = Setup::load(&CoreArgs {
            calib: None,
            hold: Hold::Hand,
            bias: None,
        })
        .expect("setup");
        let board = calfile::board().expect("board.cal");
        let no_fallback = || -> anyhow::Result<(V3, &'static str)> { panic!("not asked") };
        let (cfg, about) = setup
            .config(&hello(0x48, 0x44), no_fallback)
            .expect("config");
        assert_eq!(CalFile::from_gc(&cfg.gc()), board);
        assert!(about.contains("linked board.cal"), "{about}");
        // The thresholds and limits: the table in docs/calibration.md, which the recognize
        // firmware carries as constants (`live_config`).
        let gc = cfg.gc();
        assert_eq!(
            (
                gc.still_gyro_std_lsb,
                gc.still_accel_std_lsb,
                gc.still_gyro_rate_lsb
            ),
            (171, 123, 229)
        );
        assert_eq!(
            (gc.hold_n, gc.bias_gyro_std_lsb, gc.bias_accel_std_lsb),
            (20, 11, 16)
        );
        assert_eq!((gc.clip_lsb, gc.max_dt_us), (32700, 14423));
        assert_eq!((gc.min_stroke_us, gc.max_stroke_us), (150_000, 4_000_000));
        assert_eq!((gc.min_extent_mm, gc.max_v_end_mm_s), (20, 300));
        assert_eq!((gc.reserved0, gc.reserved1), (0, 0));

        let fallback = || Ok(([1.0, 2.0, 3.0], "fallback"));
        let (cfg, about) = setup.config(&hello(0x40, 0x40), fallback).expect("config");
        let gc = cfg.gc();
        assert_eq!(gc.accel_offset_lsb, [0; 3]);
        assert_eq!(gc.accel_gain_q30, [1 << 30; 3]);
        assert!(
            about.contains("raw (the linked board.cal is for other scales)"),
            "{about}"
        );
        assert!(
            about.contains("1.00 2.00 3.00") && about.contains("fallback"),
            "{about}"
        );
    }
}
