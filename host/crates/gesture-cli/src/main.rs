//! gesture-cli: record sessions from the stream firmware into `data/`, replay them
//! through the host build of core/, and compare against the recognize firmware's events.

mod check;

use std::io::ErrorKind;
use std::time::{Duration, Instant};

use anyhow::{Context, bail};
use clap::{Args, Parser, Subcommand};
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

fn check(port: &PortArgs, seconds: u64, print: bool) -> anyhow::Result<()> {
    let mut serial = open(port)?;
    serial.clear(ClearBuffer::Input)?;
    let mut deframer = Deframer::new();
    let mut stats = Stats::new();
    let start = Instant::now();
    let duration = Duration::from_secs(seconds);
    let mut next_report = REPORT_EVERY;
    let mut buf = [0u8; READ_BUF];
    while start.elapsed() < duration {
        let n = read_some(serial.as_mut(), &mut buf)?;
        for frame in deframer.feed(&buf[..n]) {
            handle(&mut stats, frame, print);
        }
        if start.elapsed() >= next_report {
            report(&mut stats, next_report);
            next_report += REPORT_EVERY;
        }
    }

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
    let failures = stats.failures();
    if !failures.is_empty() {
        bail!("check failed: {}", failures.join(", "));
    }
    println!("ok");
    Ok(())
}

fn main() -> anyhow::Result<()> {
    match Cli::parse().command {
        Command::Check {
            port,
            seconds,
            print,
        } => check(&port, seconds, print),
    }
}
