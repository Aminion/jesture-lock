//! The ST-LINK virtual COM port the stream firmware writes to.

use std::io::ErrorKind;
use std::time::{Duration, Instant};

use anyhow::Context;
use clap::Args;
use serialport::{ClearBuffer, SerialPort};

#[derive(Debug, Clone, Args)]
pub struct PortArgs {
    /// Serial device of the ST-LINK virtual COM port.
    #[arg(long, default_value = "/dev/ttyACM0")]
    pub port: String,
    #[arg(long, default_value_t = protocol::BAUD)]
    pub baud: u32,
}

/// Read chunk size; the port timeout, not the buffer, paces the loops.
pub const READ_BUF: usize = 256;

/// How long one read waits for bytes before returning none.
pub const PORT_TIMEOUT: Duration = Duration::from_millis(100);

/// How long to discard input after opening: the ST-LINK hands over what it buffered
/// while the port was closed (seen: ~117 s of an old stream) in the first reads.
pub const SETTLE: Duration = Duration::from_millis(300);

pub fn open(PortArgs { port, baud }: &PortArgs) -> anyhow::Result<Box<dyn SerialPort>> {
    serialport::new(port, *baud)
        .timeout(PORT_TIMEOUT)
        // Asserting DTR on open resets the MCU through the ST-LINK (seen on this Nucleo).
        .dtr_on_open(false)
        .open()
        .with_context(|| format!("opening {port}"))
}

/// Serial ports present now, by name (the ST-LINK is a `/dev/ttyACM*` on Linux).
pub fn ports() -> Vec<String> {
    serialport::available_ports()
        .map(|ps| ps.into_iter().map(|p| p.port_name).collect())
        .unwrap_or_default()
}

/// Reads what is available into `buf`; 0 if nothing came within the port timeout.
pub fn read_some(serial: &mut dyn SerialPort, buf: &mut [u8]) -> anyhow::Result<usize> {
    match serial.read(buf) {
        Ok(n) => Ok(n),
        Err(e) if e.kind() == ErrorKind::TimedOut => Ok(0),
        Err(e) => Err(e).with_context(|| format!("reading {}", serial.name().unwrap_or_default())),
    }
}

/// Drops the stale backlog: clears the input, then discards what arrives for `SETTLE`.
pub fn settle(serial: &mut dyn SerialPort) -> anyhow::Result<()> {
    serial.clear(ClearBuffer::Input)?;
    let mut buf = [0u8; READ_BUF];
    let start = Instant::now();
    while start.elapsed() < SETTLE {
        read_some(serial, &mut buf)?;
    }
    Ok(())
}
