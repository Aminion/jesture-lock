//! Viewer: a live stream or a replayed session -> the core (FFI) -> drawing. Converts raw LSB
//! to physical units (g, deg/s) for display only.

mod app;
mod capture;
mod feed;
mod scene;
mod source;

use std::path::PathBuf;

use clap::Parser;
use session::reference::{CoreArgs, Setup};

/// Configured as `gesture-cli strokes` is; the source can be changed in the window.
#[derive(Debug, Parser)]
struct Args {
    /// Session file (`.bin`) to replay at start.
    file: Option<PathBuf>,
    /// Serial port to connect to at start, instead of a file.
    #[arg(long, conflicts_with = "file")]
    port: Option<String>,
    #[arg(long, default_value_t = protocol::BAUD)]
    baud: u32,
    #[command(flatten)]
    core: CoreArgs,
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();
    let setup = Setup::load(&args.core)?;
    let start = app::Start {
        file: args.file,
        port: args.port,
        baud: args.baud,
    };
    eframe::run_native(
        "Gesture IMU viewer",
        eframe::NativeOptions::default(),
        Box::new(move |cc| Ok(Box::new(app::App::new(cc.egui_ctx.clone(), setup, start)))),
    )
    .map_err(|e| anyhow::anyhow!("{e}"))
}
