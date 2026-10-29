//! gesture-cli: record sessions from fw_stream into `data/`, replay them
//! through the host build of core/, and compare against fw_recognize events.

use std::io::{self, ErrorKind, Read, Write};
use std::time::{Duration, Instant};

use anyhow::{Context, bail};
use clap::{Parser, Subcommand};
use serialport::{ClearBuffer, SerialPort};

#[derive(Debug, Parser)]
#[command(version, about)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Print raw bytes from the serial port to stdout (bring-up, plain-text firmware output).
    Monitor {
        /// Serial device of the ST-LINK virtual COM port.
        #[arg(long, default_value = "/dev/ttyACM0")]
        port: String,
        #[arg(long, default_value_t = 115_200)]
        baud: u32,
    },
    /// Send numbered lines and check that the firmware echoes each one back (bring-up).
    Echo {
        /// Serial device of the ST-LINK virtual COM port.
        #[arg(long, default_value = "/dev/ttyACM0")]
        port: String,
        #[arg(long, default_value_t = 115_200)]
        baud: u32,
        /// Number of lines to send.
        #[arg(long, default_value_t = 100)]
        count: u32,
    },
}

/// Longest payload the firmware echoes without truncation (72-byte line minus "echo " and CRLF).
const MAX_ECHO: usize = 64;

fn open(port: &str, baud: u32) -> anyhow::Result<Box<dyn SerialPort>> {
    serialport::new(port, baud)
        .timeout(Duration::from_millis(100))
        // Asserting DTR on open resets the MCU through the ST-LINK (seen on this Nucleo).
        .dtr_on_open(false)
        .open()
        .with_context(|| format!("opening {port}"))
}

/// Printable ASCII, cycling through the whole range, `len` bytes, starting at `seed`.
fn payload(seed: u32, len: usize) -> String {
    (0..len)
        .map(|i| char::from(b' ' + ((seed as usize + i) % 95) as u8))
        .collect()
}

/// Next line without CR/LF; a partial line stays in `pending` across calls.
/// `None` if `deadline` passes first.
fn read_line(
    serial: &mut dyn SerialPort,
    pending: &mut Vec<u8>,
    deadline: Instant,
) -> anyhow::Result<Option<String>> {
    loop {
        if let Some(end) = pending.iter().position(|&b| b == b'\n') {
            let line: Vec<u8> = pending.drain(..=end).collect();
            let text = String::from_utf8_lossy(&line);
            return Ok(Some(text.trim_end_matches(['\r', '\n']).to_owned()));
        }
        if Instant::now() >= deadline {
            return Ok(None);
        }
        let mut buf = [0u8; 256];
        match serial.read(&mut buf) {
            Ok(n) => pending.extend_from_slice(&buf[..n]),
            Err(e) if e.kind() == ErrorKind::TimedOut => {}
            Err(e) => return Err(e).context("reading serial port"),
        }
    }
}

/// Reads and discards until nothing arrives for `quiet`.
fn drain(serial: &mut dyn SerialPort, quiet: Duration) -> anyhow::Result<()> {
    serial.clear(ClearBuffer::Input)?;
    let mut last = Instant::now();
    let mut buf = [0u8; 256];
    while last.elapsed() < quiet {
        match serial.read(&mut buf) {
            Ok(_) => last = Instant::now(),
            Err(e) if e.kind() == ErrorKind::TimedOut => {}
            Err(e) => return Err(e).context("reading serial port"),
        }
    }
    Ok(())
}

fn echo(port: &str, baud: u32, count: u32) -> anyhow::Result<()> {
    let mut serial = open(port, baud)?;
    // The ST-LINK keeps a backlog of MCU output from while the port was closed and sends
    // it in a burst on open. Drop input until it goes quiet: status lines are 500 ms apart.
    drain(serial.as_mut(), Duration::from_millis(200))?;
    let mut pending = Vec::new();
    let mut rtts = Vec::new();
    let mut skipped = 0u32;

    for i in 0..count {
        let prefix = format!("ping {i} ");
        let sent = prefix.clone() + &payload(i, (i as usize * 7) % (MAX_ECHO - prefix.len() + 1));
        let start = Instant::now();
        serial.write_all(format!("{sent}\n").as_bytes())?;
        let deadline = start + Duration::from_secs(1);
        loop {
            let Some(line) = read_line(serial.as_mut(), &mut pending, deadline)? else {
                bail!("line {i}: no echo within 1 s (sent {sent:?})");
            };
            // Status lines, and a garbled first line left over in the VCP, are not echoes.
            let Some(got) = line.strip_prefix("echo ") else {
                if skipped == 0 {
                    println!("skip: {line}");
                }
                skipped += 1;
                continue;
            };
            if got != sent {
                bail!("line {i}: sent {sent:?}, got {line:?}");
            }
            break;
        }
        rtts.push(start.elapsed());
    }

    let min = rtts.iter().min().copied().unwrap_or_default();
    let max = rtts.iter().max().copied().unwrap_or_default();
    let avg = rtts.iter().sum::<Duration>() / count.max(1);
    println!(
        "echo: {count}/{count} lines ok, rtt min {min:.1?} avg {avg:.1?} max {max:.1?}, {skipped} other lines skipped"
    );
    Ok(())
}

fn monitor(port: &str, baud: u32) -> anyhow::Result<()> {
    let mut serial = open(port, baud)?;
    let mut stdout = io::stdout().lock();
    let mut buf = [0u8; 256];
    loop {
        match serial.read(&mut buf) {
            Ok(n) => {
                stdout.write_all(&buf[..n])?;
                stdout.flush()?;
            }
            Err(e) if e.kind() == ErrorKind::TimedOut => {}
            Err(e) => return Err(e).with_context(|| format!("reading {port}")),
        }
    }
}

fn main() -> anyhow::Result<()> {
    match Cli::parse().command {
        Command::Monitor { port, baud } => monitor(&port, baud),
        Command::Echo { port, baud, count } => echo(&port, baud, count),
    }
}
