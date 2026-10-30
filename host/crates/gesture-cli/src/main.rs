//! gesture-cli: record sessions from the stream firmware into `data/`, replay them
//! through the host build of core/, and compare against the recognize firmware's events.

mod calib;
mod check;
mod path;
mod stats;

use std::fs::{self, File};
use std::io::{BufWriter, ErrorKind, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use anyhow::{Context, bail};
use clap::{Args, Parser, Subcommand, ValueEnum};
use protocol::{Deframer, Message};
use serialport::{ClearBuffer, SerialPort};

use crate::check::Stats;

#[derive(Debug, Parser)]
#[command(version, about)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Read the stream and check it: frame errors, lost frames, missed samples, MCU counters.
    /// Exits non-zero if anything went wrong.
    Check {
        #[command(flatten)]
        port: PortArgs,
        /// How long to read.
        #[arg(long, default_value_t = 10)]
        seconds: u64,
        /// Print every decoded message.
        #[arg(long)]
        print: bool,
    },
    /// Record the raw stream to `<dir>/<time>_<label>.bin`, with a `.toml` next to it
    /// (label, note, stream health). The file holds the bytes as received
    /// (`docs/protocol.md` framing), so it replays through the same deframer.
    /// Numbers from a recorded session: per-axis bias, noise and drift, and how far a
    /// stroke would drift on noise alone. For the rest runs.
    Stats {
        /// Session file (`.bin`).
        file: PathBuf,
    },
    /// Accelerometer offset and scale per axis from a tumble session: the board held still
    /// in many orientations, each still segment must read 1 g.
    Calib {
        /// Session file (`.bin`).
        file: PathBuf,
    },
    /// Per-window spread and rate: where the board was still, and how still.
    Timeline {
        /// Session file (`.bin`).
        file: PathBuf,
        /// Window length, s.
        #[arg(long, default_value_t = 0.25)]
        window: f64,
    },
    /// Stroke paths (hold -> move -> hold), float reference for the `core/` spec.
    Path {
        /// Session file (`.bin`).
        file: PathBuf,
        /// Tumble session to calibrate the accelerometer with (`calib`).
        #[arg(long)]
        calib: Option<PathBuf>,
        /// Still thresholds: on a table or in the hand.
        #[arg(long, value_enum, default_value_t = Hold::Hand)]
        hold: Hold,
        /// Take the gyro bias from the whole session (median) instead of each hold.
        #[arg(long)]
        session_bias: bool,
    },
    Record {
        #[command(flatten)]
        port: PortArgs,
        /// Short name of what was done: lowercase letters, digits and `-`.
        label: String,
        /// Free text: how the board was held, what moved, anything odd.
        #[arg(long, default_value = "")]
        note: String,
        /// How long to record.
        #[arg(long, default_value_t = 10)]
        seconds: u64,
        /// Output directory [default: data/sessions/ of this repository].
        #[arg(long)]
        dir: Option<PathBuf>,
    },
}

/// Where the board rests between strokes.
#[derive(Clone, Copy, Debug, ValueEnum)]
enum Hold {
    Hand,
    Table,
}

#[derive(Debug, Args)]
struct PortArgs {
    /// Serial device of the ST-LINK virtual COM port.
    #[arg(long, default_value = "/dev/ttyACM0")]
    port: String,
    #[arg(long, default_value_t = protocol::BAUD)]
    baud: u32,
}

/// Read chunk size; the port timeout, not the buffer, paces the loops.
const READ_BUF: usize = 256;

/// How long one read waits for bytes before returning none.
const PORT_TIMEOUT: Duration = Duration::from_millis(100);

/// How long to discard input after opening: the ST-LINK hands over what it buffered
/// while the port was closed (seen: ~117 s of an old stream) in the first reads.
const SETTLE: Duration = Duration::from_millis(300);

/// How often `check` prints a progress line (PC time: it only paces the output).
const REPORT_EVERY: Duration = Duration::from_secs(1);

fn open(PortArgs { port, baud }: &PortArgs) -> anyhow::Result<Box<dyn SerialPort>> {
    serialport::new(port, *baud)
        .timeout(PORT_TIMEOUT)
        // Asserting DTR on open resets the MCU through the ST-LINK (seen on this Nucleo).
        .dtr_on_open(false)
        .open()
        .with_context(|| format!("opening {port}"))
}

/// Reads what is available into `buf`; 0 if nothing came within the port timeout.
fn read_some(serial: &mut dyn SerialPort, buf: &mut [u8]) -> anyhow::Result<usize> {
    match serial.read(buf) {
        Ok(n) => Ok(n),
        Err(e) if e.kind() == ErrorKind::TimedOut => Ok(0),
        Err(e) => Err(e).with_context(|| format!("reading {}", serial.name().unwrap_or_default())),
    }
}

/// Feeds one deframed frame to `stats`, printing events (and messages if `print`).
fn handle(stats: &mut Stats, frame: Result<Vec<u8>, protocol::FrameError>, print: bool) {
    let payload = match frame {
        Ok(p) => p,
        Err(e) => {
            if print {
                println!("frame error: {e}");
            }
            return stats.frame_error(e);
        }
    };
    match Message::decode(&payload) {
        Err(e) => {
            if print {
                println!("message error: {e}");
            }
            stats.message_error(e);
        }
        Ok(None) => stats.skipped(),
        Ok(Some(m)) => {
            if print {
                println!("{m:?}");
            }
            if let Some(event) = stats.message(&m) {
                println!("{event}");
            }
        }
    }
}

fn report(stats: &mut Stats, elapsed: Duration) {
    let secs = elapsed.as_secs();
    if !stats.started() {
        println!("t={secs}s waiting for Hello");
        return;
    }
    let iv = stats.take_interval();
    let dt = iv
        .dt
        .map_or("-".to_owned(), |(lo, hi)| format!("{lo}..{hi} us"));
    let t = stats.totals();
    let m = &t.mcu;
    println!(
        "t={secs}s samples {} dt {dt} | frame_err {} lost {} t_gaps {} | mcu missed {} stuck {} i2c {} rx {}",
        iv.samples,
        t.frame_errors.total(),
        t.lost,
        t.t_gaps,
        m.missed,
        m.stuck,
        m.i2c_err,
        m.rx_err
    );
}

/// Reads the port for `seconds`, checking the stream as it goes; every byte received also
/// goes to `sink`.
fn read_stream(
    port: &PortArgs,
    seconds: u64,
    print: bool,
    mut sink: Option<&mut dyn Write>,
) -> anyhow::Result<Stats> {
    let mut serial = open(port)?;
    serial.clear(ClearBuffer::Input)?;
    let mut buf = [0u8; READ_BUF];
    let settle = Instant::now();
    while settle.elapsed() < SETTLE {
        read_some(serial.as_mut(), &mut buf)?;
    }
    let mut deframer = Deframer::new();
    let mut stats = Stats::new();
    let start = Instant::now();
    let duration = Duration::from_secs(seconds);
    let mut next_report = REPORT_EVERY;
    while start.elapsed() < duration {
        let n = read_some(serial.as_mut(), &mut buf)?;
        if let Some(sink) = sink.as_mut() {
            sink.write_all(&buf[..n])?;
        }
        for frame in deframer.feed(&buf[..n]) {
            handle(&mut stats, frame, print);
        }
        if start.elapsed() >= next_report {
            report(&mut stats, next_report);
            next_report += REPORT_EVERY;
        }
    }
    Ok(stats)
}

fn print_totals(stats: &Stats) {
    let t = stats.totals();
    let e = &t.frame_errors;
    let rate = t.rate_hz().map_or("-".to_owned(), |r| format!("{r:.2} Hz"));
    println!(
        "total: {} samples ({rate}, MCU time), frame errors {} (cobs {} short {} long {} crc {}), \
         message errors {}, skipped {}, lost {}, t_gaps {}, resets {} | mcu missed {} stuck {} i2c {} rx {}",
        t.samples,
        e.total(),
        e.cobs,
        e.too_short,
        e.too_long,
        e.crc,
        t.message_errors,
        t.skipped,
        t.lost,
        t.t_gaps,
        t.resets,
        t.mcu.missed,
        t.mcu.stuck,
        t.mcu.i2c_err,
        t.mcu.rx_err
    );
}

fn check(port: &PortArgs, seconds: u64, print: bool) -> anyhow::Result<()> {
    let stats = read_stream(port, seconds, print, None)?;
    print_totals(&stats);
    let failures = stats.failures();
    if !failures.is_empty() {
        bail!("check failed: {}", failures.join(", "));
    }
    println!("ok");
    Ok(())
}

/// `data/sessions/` of this repository.
fn default_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../data/sessions")
}

fn valid_label(label: &str) -> bool {
    !label.is_empty()
        && label
            .bytes()
            .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'-')
}

/// A TOML basic string. Rust's escapes of `"`, `\` and \t \r \n match TOML; other
/// control characters are rejected before this.
fn toml_str(s: &str) -> String {
    format!("{s:?}")
}

struct Session<'a> {
    label: &'a str,
    note: &'a str,
    seconds: u64,
    started: &'a str,
}

fn sidecar(session: &Session<'_>, stats: &Stats) -> String {
    let t = stats.totals();
    let rate = t.rate_hz().map_or("0.0".to_owned(), |r| format!("{r:.3}"));
    let failures: Vec<String> = stats.failures().iter().map(|f| toml_str(f)).collect();
    format!(
        "label = {}\nnote = {}\nstarted = {}\nseconds = {}\n\n[stream]\n\
         samples = {}\nrate_hz = {rate}\nfailures = [{}]\n",
        toml_str(session.label),
        toml_str(session.note),
        toml_str(session.started),
        session.seconds,
        t.samples,
        failures.join(", ")
    )
}

fn record(port: &PortArgs, session: &Session<'_>, dir: &Path) -> anyhow::Result<()> {
    if !valid_label(session.label) {
        bail!("label must be lowercase letters, digits and '-'");
    }
    if session
        .note
        .chars()
        .any(|c| c.is_control() && !matches!(c, '\t' | '\n'))
    {
        bail!("note must not contain control characters");
    }
    fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;
    let dir = fs::canonicalize(dir)?;
    let stem = dir.join(format!(
        "{}_{}",
        jiff::Zoned::now().strftime("%Y%m%d-%H%M%S"),
        session.label
    ));
    let bin = stem.with_extension("bin");
    let file = File::create_new(&bin).with_context(|| format!("creating {}", bin.display()))?;
    let mut out = BufWriter::new(file);
    println!("recording {}s to {}", session.seconds, bin.display());
    let stats = read_stream(port, session.seconds, false, Some(&mut out))?;
    out.flush()?;
    print_totals(&stats);

    let toml = stem.with_extension("toml");
    fs::write(&toml, sidecar(session, &stats))
        .with_context(|| format!("writing {}", toml.display()))?;
    println!("wrote {} and {}", bin.display(), toml.display());
    let failures = stats.failures();
    if !failures.is_empty() {
        bail!(
            "stream had problems (kept, listed in the .toml): {}",
            failures.join(", ")
        );
    }
    Ok(())
}

fn main() -> anyhow::Result<()> {
    match Cli::parse().command {
        Command::Check {
            port,
            seconds,
            print,
        } => check(&port, seconds, print),
        Command::Stats { file } => {
            let bytes = fs::read(&file).with_context(|| format!("reading {}", file.display()))?;
            print!("{}", stats::report(&stats::load(&bytes)?)?);
            Ok(())
        }
        Command::Calib { file } => {
            let bytes = fs::read(&file).with_context(|| format!("reading {}", file.display()))?;
            print!("{}", calib::report(&stats::load(&bytes)?)?);
            Ok(())
        }
        Command::Timeline { file, window } => {
            let bytes = fs::read(&file).with_context(|| format!("reading {}", file.display()))?;
            print!("{}", stats::timeline(&stats::load(&bytes)?, window)?);
            Ok(())
        }
        Command::Path {
            file,
            calib,
            hold,
            session_bias,
        } => {
            let fit = calib
                .map(|c| -> anyhow::Result<_> {
                    let bytes = fs::read(&c).with_context(|| format!("reading {}", c.display()))?;
                    calib::fit_session(&stats::load(&bytes)?)
                })
                .transpose()?;
            let bytes = fs::read(&file).with_context(|| format!("reading {}", file.display()))?;
            print!(
                "{}",
                path::report(
                    &stats::load(&bytes)?,
                    fit.as_ref(),
                    match hold {
                        Hold::Hand => calib::HAND,
                        Hold::Table => calib::TABLE,
                    },
                    session_bias,
                )?
            );
            Ok(())
        }
        Command::Record {
            port,
            label,
            note,
            seconds,
            dir,
        } => {
            let started = jiff::Zoned::now()
                .strftime("%Y-%m-%dT%H:%M:%S%:z")
                .to_string();
            let session = Session {
                label: &label,
                note: &note,
                seconds,
                started: &started,
            };
            record(&port, &session, &dir.unwrap_or_else(default_dir))
        }
    }
}
