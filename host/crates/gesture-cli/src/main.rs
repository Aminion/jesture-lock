//! gesture-cli: record sessions from fw_stream into `data/`, replay them
//! through the host build of core/, and compare against fw_recognize events.

use std::io::{self, ErrorKind, Read, Write};
use std::time::Duration;

use anyhow::Context;
use clap::{Parser, Subcommand};

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
}

fn monitor(port: &str, baud: u32) -> anyhow::Result<()> {
    let mut serial = serialport::new(port, baud)
        .timeout(Duration::from_millis(100))
        .open()
        .with_context(|| format!("opening {port}"))?;
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
    }
}
