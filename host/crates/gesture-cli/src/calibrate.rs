//! `calibrate`: the guided calibration run of `docs/calibration.md`. The steps themselves
//! are `session::calibrate`; this is the port, the terminal and the files.

use std::fmt::Write as _;
use std::fs::{self, File};
use std::io::{self, BufRead, BufWriter, IsTerminal, Write};
use std::path::{Path, PathBuf};
use std::sync::mpsc::{self, Receiver};
use std::time::{Duration, Instant};

use anyhow::{Context, bail};
use clap::Args;
use protocol::{Deframer, FrameError, Message};
use serialport::SerialPort;
use session::calfile::{CalFile, describe_scales};
use session::calibrate::{
    COVERAGE_G, DIRECTIONS, Fatal, MIN_HOLDS, Measurement, Note, Params, Phase, Run, Stage,
};
use session::check::Stats;
use session::config::{Q8, Q30};
use session::serial::{self, PortArgs, READ_BUF};

use crate::{Session, default_dir, sidecar};

#[derive(Debug, Args)]
pub struct CalibrateArgs {
    #[command(flatten)]
    port: PortArgs,
    /// Run the steps on the recorded session of an earlier run instead of the port: no
    /// prompts, no retries.
    #[arg(long)]
    replay: Option<PathBuf>,
    /// Calibration file to write [default: data/calibration/board.cal of this repository;
    /// with --replay: none].
    #[arg(long)]
    out: Option<PathBuf>,
    /// Directory for the session of the run [default: data/sessions/ of this repository].
    #[arg(long)]
    dir: Option<PathBuf>,
}

/// PC time: these must fire when nothing arrives.
const HELLO_TIMEOUT: Duration = Duration::from_secs(5);
const REST_TIMEOUT: Duration = Duration::from_secs(30);
/// After connect: no sample for this long means the IMU stopped (the firmware does not
/// start it again).
const SAMPLES_TIMEOUT: Duration = Duration::from_secs(2);

const BAR: usize = 20;
/// The progress line is cut to this, to stay on one row of an 80-column terminal.
const PROGRESS_COLUMNS: usize = 79;
/// Readings on 2026-10-30 (`docs/todo.md`), to compare a rest against.
const REST_SEEN: &str = "seen on 2026-10-30: gyro 0.05 dps, accel 0.5 to 1.6 mg";

/// `data/calibration/board.cal` of this repository.
fn default_out() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../data/calibration/board.cal")
}

/// A path as the lines of the run name it: the file name only. The lines are saved in the
/// session note, which is committed, so this machine's directories stay out of them.
fn shown(path: &Path) -> String {
    path.file_name().map_or_else(
        || path.display().to_string(),
        |n| n.to_string_lossy().into_owned(),
    )
}

/// Lines for the terminal, with one progress line redrawn in place under them. Every
/// line also goes to `log`, the note of the saved session.
struct Ui {
    tty: bool,
    progress: bool,
    log: String,
}

impl Ui {
    fn new() -> Self {
        Self {
            tty: io::stdout().is_terminal(),
            progress: false,
            log: String::new(),
        }
    }

    fn clear(&mut self) {
        if self.progress {
            print!("\r\x1b[2K");
            self.progress = false;
        }
    }

    fn line(&mut self, s: &str) {
        self.clear();
        println!("{s}");
        self.log.push_str(s);
        self.log.push('\n');
    }

    /// Redraws the progress line (terminals only).
    fn progress(&mut self, s: &str) {
        if self.tty {
            // One row only: a wrapped line would scroll at every redraw.
            let row: String = s.chars().take(PROGRESS_COLUMNS).collect();
            print!("\r\x1b[2K{row}");
            // A failed flush only delays the redraw.
            let _ = io::stdout().flush();
            self.progress = true;
        }
    }
}

fn max3(v: [f64; 3]) -> f64 {
    v.into_iter().fold(0.0, f64::max)
}

fn clock(us: u64) -> String {
    let s = us / 1_000_000;
    format!("{:02}:{:02}", s / 60, s % 60)
}

fn bar(done_us: u64, of_us: u64) -> String {
    let filled = (done_us * BAR as u64 / of_us.max(1)) as usize;
    let s = |us: u64| us as f64 / 1e6;
    format!(
        "[{}{}] {:.1}/{:.1} s, {:.1} s left",
        "#".repeat(filled),
        ".".repeat(BAR - filled),
        s(done_us),
        s(of_us),
        s(of_us - done_us)
    )
}

/// "x has not pointed up yet" for a direction of the coverage rule (`+x`): an axis reads
/// +1 g when it points up.
fn hint(direction: &str) -> String {
    let (sign, axis) = direction.split_at(1);
    let way = if sign == "+" { "up" } else { "down" };
    format!("{axis} has not pointed {way} yet")
}

/// The holds so far: coverage, what is missing, the fit.
fn holds_state(run: &Run) -> Vec<String> {
    let holds = run.holds();
    let coverage: Vec<String> = holds
        .coverage()
        .iter()
        .zip(DIRECTIONS)
        .map(|(g, d)| format!("{d} {g:.2}{}", if *g >= COVERAGE_G { "*" } else { " " }))
        .collect();
    let mut out = vec![format!(
        "    coverage, g (* = at least {COVERAGE_G}): {}",
        coverage.join(" ")
    )];
    let missing = holds.missing();
    if !missing.is_empty() {
        let hints: Vec<String> = missing.iter().map(|d| hint(d)).collect();
        out.push(format!("    {}", hints.join("; ")));
    }
    if let Some(s) = holds.solve() {
        let (worst, worst_mg) = s.worst();
        out.push(format!(
            "    fit: {:.1} mg rms (raw {:.1}), worst is hold {} with {worst_mg:.1} mg",
            s.after_mg,
            s.before_mg,
            worst + 1
        ));
    }
    out
}

fn hold_line(n: usize, m: &Measurement) -> String {
    format!(
        "  hold {n:>2}: accel {:+.3} {:+.3} {:+.3} g, gyro std {:.1} dps, accel std {:.1} mg",
        m.accel[0] / 1e3,
        m.accel[1] / 1e3,
        m.accel[2] / 1e3,
        max3(m.gyro_std),
        max3(m.accel_std)
    )
}

const HOLDS_HELP: &str = "    commands, then Enter: u drop the last hold, d N drop hold N, \
                          r drop all, f finish, q quit";

/// What to say about a note.
fn describe(run: &Run, note: &Note) -> Vec<String> {
    match note {
        Note::Connected { hello, rate_hz } => vec![
            format!(
                "[1/5] connect: ok. stream firmware, {}, {rate_hz:.1} Hz",
                describe_scales(hello)
            ),
            "[2/5] rest: put the board on the table and do not touch it.".to_owned(),
        ],
        Note::Rest(m) => vec![
            format!(
                "[2/5] rest: ok. gyro bias {:+.3} {:+.3} {:+.3} dps over {:.1} s",
                m.gyro[0],
                m.gyro[1],
                m.gyro[2],
                (m.t_us.1 - m.t_us.0) as f64 / 1e6
            ),
            format!(
                "    std: gyro {:.3} {:.3} {:.3} dps, accel {:.2} {:.2} {:.2} mg ({REST_SEEN})",
                m.gyro_std[0],
                m.gyro_std[1],
                m.gyro_std[2],
                m.accel_std[0],
                m.accel_std[1],
                m.accel_std[2]
            ),
            "[3/5] holds: turn the board to a new orientation and keep it still. Lean it \
             against something if you like. Exact angles do not matter."
                .to_owned(),
            format!(
                "    needed: at least {MIN_HOLDS} holds, each axis once up and once down \
                 (within about 30 degrees)"
            ),
            "    the position on the table counts as the first hold if the board stays there"
                .to_owned(),
            HOLDS_HELP.to_owned(),
        ],
        Note::Hold(index) => {
            let mut out = vec![hold_line(index + 1, &run.holds().list()[*index])];
            out.extend(holds_state(run));
            out
        }
        Note::Rejected(_, close) => vec![format!(
            "  too close to hold {} ({:.0} degrees), turn it further",
            close.to + 1,
            close.angle_deg
        )],
        Note::Moved => vec!["  moved during the measurement: hold it again".to_owned()],
        Note::Discarded(fault) => vec![format!("  measurement discarded: {fault}")],
    }
}

fn progress_line(run: &Run, waited: Duration) -> String {
    let elapsed = clock(run.step_elapsed_us());
    let phase = |moving: &str| match run.phase() {
        Phase::Moving => moving.to_owned(),
        Phase::Settling => "settling".to_owned(),
        Phase::Measuring { done_us, of_us } => bar(done_us, of_us),
        Phase::Spent => "taken: turn the board".to_owned(),
    };
    match run.stage() {
        Stage::Connect => format!(
            "[1/5] connect  {:.0} s  {}",
            waited.as_secs_f64(),
            if run.hello().is_some() {
                "measuring the sample rate"
            } else {
                "waiting for Hello"
            }
        ),
        Stage::Rest => format!("[2/5] rest  {elapsed}  {}", phase("waiting for stillness")),
        Stage::Holds => {
            let missing = run.holds().missing();
            format!(
                "[3/5] holds {}/{MIN_HOLDS}  {elapsed}  {}{}",
                run.holds().list().len(),
                phase("turn and hold still"),
                if missing.is_empty() {
                    String::new()
                } else {
                    format!("  | missing {}", missing.join(" "))
                }
            )
        }
        Stage::Finished => String::new(),
    }
}

/// One frame into the stream health and the run.
fn feed(
    run: &mut Run,
    stats: &mut Stats,
    frame: Result<Vec<u8>, FrameError>,
) -> Result<Option<Note>, Fatal> {
    let payload = match frame {
        Ok(p) => p,
        Err(e) => {
            stats.frame_error(e);
            return Ok(run.bad_frame());
        }
    };
    match Message::decode(&payload) {
        Err(e) => {
            stats.message_error(e);
            Ok(run.bad_frame())
        }
        Ok(None) => {
            stats.skipped();
            Ok(None)
        }
        Ok(Some(m)) => {
            stats.message(&m);
            run.message(&m)
        }
    }
}

fn gate_lines(run: &Run) -> Vec<String> {
    run.holds()
        .gates()
        .iter()
        .map(|g| {
            format!(
                "    {} {:<13} {}",
                if g.pass { "pass" } else { "FAIL" },
                g.name,
                g.detail
            )
        })
        .collect()
}

/// Step 4: the holds, the fitted values as they go into the file, the gates, and the
/// difference from the file at `current`.
fn report(run: &Run, current: Option<&Path>) -> anyhow::Result<(CalFile, String)> {
    let (Some(cfg), Some(solution)) = (run.config(), run.holds().solve()) else {
        bail!("no fit");
    };
    let file = CalFile::from_gc(&cfg.gc());
    let mut out = String::from("[4/5] fit and gates\n");
    writeln!(
        out,
        "    hold      accel x/y/z, g      |a|-1 raw   fit, mg   gyro std  accel std"
    )?;
    for (i, (m, r)) in run
        .holds()
        .list()
        .iter()
        .zip(&solution.residual_mg)
        .enumerate()
    {
        let a = m.accel.map(|x| x / 1e3);
        let raw = (a.iter().map(|x| x * x).sum::<f64>().sqrt() - 1.0) * 1e3;
        writeln!(
            out,
            "    {:>4}  {:>+7.3} {:>+7.3} {:>+7.3}   {raw:>+7.1}   {r:>+7.1}   {:>6.1}   {:>6.1}",
            i + 1,
            a[0],
            a[1],
            a[2],
            max3(m.gyro_std),
            max3(m.accel_std)
        )?;
    }
    let mg = |lsb: f64| lsb * cfg.sa / 1e3;
    let dps = |q8: f64| q8 / Q8 * cfg.sg / 1e6;
    let gain = |q30: i32| f64::from(q30) / Q30;
    writeln!(
        out,
        "    axis  accel offset, mg (LSB)   gain     gyro bias, dps (LSB)"
    )?;
    for (i, axis) in ["x", "y", "z"].iter().enumerate() {
        writeln!(
            out,
            "      {axis}     {:>+7.1} ({:>+5})       {:.4}    {:>+7.3} ({:>+7.1})",
            mg(f64::from(file.accel_offset_lsb[i])),
            file.accel_offset_lsb[i],
            gain(file.accel_gain_q30[i]),
            dps(f64::from(file.gyro_bias_q8[i])),
            f64::from(file.gyro_bias_q8[i]) / Q8
        )?;
    }
    writeln!(
        out,
        "    |a| error, rms over the holds: {:.1} mg raw -> {:.1} mg fitted",
        solution.before_mg, solution.after_mg
    )?;
    for line in gate_lines(run) {
        writeln!(out, "{line}")?;
    }
    if let Some(path) = current {
        match fs::read(path).map(|b| CalFile::parse(&b)) {
            Err(_) => writeln!(out, "    no current {} to compare with", shown(path))?,
            Ok(Err(e)) => writeln!(out, "    current {} is not valid: {e}", shown(path))?,
            Ok(Ok(old)) if !old.applies_to(file.ctrl1_xl, file.ctrl2_g) => writeln!(
                out,
                "    current {} is for other scales ({})",
                shown(path),
                old.scales()
            )?,
            Ok(Ok(old)) => {
                let d = |f: &dyn Fn(usize) -> f64| {
                    let v: [f64; 3] = std::array::from_fn(f);
                    format!("{:+.3} {:+.3} {:+.3}", v[0], v[1], v[2])
                };
                writeln!(
                    out,
                    "    change from {} (shown, not judged): offset {} mg, gain {}, bias {} dps",
                    shown(path),
                    d(&|i| mg(
                        f64::from(file.accel_offset_lsb[i]) - f64::from(old.accel_offset_lsb[i])
                    )),
                    d(&|i| gain(file.accel_gain_q30[i]) - gain(old.accel_gain_q30[i])),
                    d(&|i| dps(f64::from(file.gyro_bias_q8[i]) - f64::from(old.gyro_bias_q8[i])))
                )?;
            }
        }
    }
    Ok((file, out.trim_end().to_owned()))
}

/// Step 5: writes the file, reads it back and checks it.
fn write_file(file: &CalFile, out: &Path) -> anyhow::Result<String> {
    file.validate()?;
    if let Some(dir) = out.parent() {
        fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;
    }
    fs::write(out, file.to_bytes()).with_context(|| format!("writing {}", out.display()))?;
    let back = fs::read(out).with_context(|| format!("reading back {}", out.display()))?;
    if CalFile::parse(&back)? != *file {
        bail!("{} reads back differently", out.display());
    }
    Ok(format!(
        "[5/5] wrote {} ({} B, for {}). Builds that link it pick it up at the next build; \
         the tools take it now with --calib.",
        shown(out),
        back.len(),
        file.scales()
    ))
}

fn replay(session: &Path, out: Option<&Path>) -> anyhow::Result<()> {
    let bytes = fs::read(session).with_context(|| format!("reading {}", session.display()))?;
    let mut ui = Ui::new();
    let mut run = Run::new(Params::default());
    let mut stats = Stats::new();
    for frame in Deframer::new().feed(&bytes) {
        if let Some(note) = feed(&mut run, &mut stats, frame)? {
            for line in describe(&run, &note) {
                ui.line(&line);
            }
        }
        if run.stage() == Stage::Finished {
            break;
        }
    }
    if run.stage() != Stage::Finished {
        if run.stage() == Stage::Holds {
            for line in gate_lines(&run) {
                ui.line(&line);
            }
        }
        bail!("the session ends in the {:?} step", run.stage());
    }
    let (file, text) = report(&run, out)?;
    ui.line(&text);
    if let Some(out) = out {
        ui.line(&write_file(&file, out)?);
    }
    Ok(())
}

/// The port, the session file and the stream health of a live run.
struct Live {
    serial: Box<dyn SerialPort>,
    sink: BufWriter<File>,
    deframer: Deframer,
    stats: Stats,
    commands: Receiver<String>,
}

impl Live {
    /// Reads what the port has; every byte goes to the session file.
    fn read(&mut self) -> anyhow::Result<Vec<Result<Vec<u8>, FrameError>>> {
        let mut buf = [0u8; READ_BUF];
        let n = serial::read_some(self.serial.as_mut(), &mut buf)?;
        self.sink.write_all(&buf[..n])?;
        Ok(self.deframer.feed(&buf[..n]).collect())
    }

    /// Waits for one of `answers` (first letters), still reading and recording the port.
    /// The run is not fed meanwhile. `None` when the input is closed.
    fn ask(&mut self, ui: &mut Ui, question: &str, answers: &str) -> anyhow::Result<Option<char>> {
        ui.line(question);
        loop {
            for frame in self.read()? {
                match frame.map(|p| Message::decode(&p)) {
                    Err(e) => self.stats.frame_error(e),
                    Ok(Err(e)) => self.stats.message_error(e),
                    Ok(Ok(None)) => self.stats.skipped(),
                    Ok(Ok(Some(m))) => {
                        self.stats.message(&m);
                    }
                }
            }
            match self.commands.try_recv() {
                Ok(line) => {
                    if let Some(c) = line.trim().chars().next()
                        && answers.contains(c)
                    {
                        return Ok(Some(c));
                    }
                    ui.line(question);
                }
                Err(mpsc::TryRecvError::Empty) => {}
                Err(mpsc::TryRecvError::Disconnected) => return Ok(None),
            }
        }
    }
}

/// A command of the holds step. False: quit.
fn command(run: &mut Run, ui: &mut Ui, line: &str) -> bool {
    let mut words = line.split_whitespace();
    let dropped = match (words.next(), words.next()) {
        (None, _) => return true,
        (Some("q"), _) => return false,
        (Some("u"), _) => {
            let n = run.holds().list().len();
            n > 0 && run.drop_hold(n - 1)
        }
        (Some("d"), Some(n)) => n
            .parse::<usize>()
            .is_ok_and(|n| n > 0 && run.drop_hold(n - 1)),
        (Some("r"), _) => {
            run.clear_holds();
            true
        }
        (Some("f"), _) => {
            for l in gate_lines(run) {
                ui.line(&l);
            }
            if !run.finish() {
                ui.line("    not finished: the gates marked FAIL have to pass");
            }
            return true;
        }
        _ => {
            ui.line(HOLDS_HELP);
            return true;
        }
    };
    if dropped {
        ui.line(&format!(
            "  dropped; {} holds left",
            run.holds().list().len()
        ));
        for l in holds_state(run) {
            ui.line(&l);
        }
        run.finish();
    } else {
        ui.line("  no such hold");
    }
    true
}

/// Steps 1 to 3. True when the gates passed, false when the user quit.
fn steps(live: &mut Live, run: &mut Run, ui: &mut Ui) -> anyhow::Result<bool> {
    let mut step_start = Instant::now();
    // The sample count and when it last grew, to notice a stream that went quiet.
    let mut last_sample = (0, Instant::now());
    loop {
        let stage = run.stage();
        let mut failure = None;
        for frame in live.read()? {
            match feed(run, &mut live.stats, frame) {
                Ok(Some(note)) => {
                    for line in describe(run, &note) {
                        ui.line(&line);
                    }
                }
                Ok(None) => {}
                // Before the rest there is nothing to lose: the step can start again.
                Err(fatal) if stage == Stage::Connect => {
                    failure = Some(fatal.to_string());
                    break;
                }
                Err(fatal) => return Err(fatal.into()),
            }
        }
        if run.stage() != stage {
            step_start = Instant::now();
        }
        let samples = live.stats.totals().samples;
        if samples != last_sample.0 {
            last_sample = (samples, Instant::now());
        } else if run.stage() != Stage::Connect && last_sample.1.elapsed() > SAMPLES_TIMEOUT {
            let mcu = live.stats.totals().mcu;
            bail!(
                "the board stopped sending samples (MCU counters during the run: missed {}, \
                 stuck {}, I2C errors {}): the IMU lost power or the bus for a moment. Check \
                 its wires, reset the board and run again",
                mcu.missed,
                mcu.stuck,
                mcu.i2c_err
            );
        }
        if run.stage() == Stage::Finished {
            return Ok(true);
        }
        let waited = step_start.elapsed();
        match run.stage() {
            Stage::Connect if waited > HELLO_TIMEOUT => {
                failure = Some(format!(
                    "{} in {} s: {}",
                    if run.hello().is_some() {
                        "a Hello but no samples"
                    } else {
                        "no Hello"
                    },
                    HELLO_TIMEOUT.as_secs(),
                    if run.hello().is_some() {
                        "the IMU stopped answering; reset the board (`gesture-cli check` shows \
                         the counters)"
                    } else {
                        "is the board connected and running the stream firmware?"
                    }
                ));
            }
            Stage::Rest
                if waited > REST_TIMEOUT && !matches!(run.phase(), Phase::Measuring { .. }) =>
            {
                failure = Some(format!(
                    "the board was not still for the whole measurement within {} s: put it on \
                     a firm table, away from the keyboard and cable pull",
                    REST_TIMEOUT.as_secs()
                ));
            }
            _ => {}
        }
        if let Some(why) = failure {
            ui.line(&format!("step failed: {why}"));
            match live.ask(ui, "r + Enter: retry the step, q + Enter: quit", "rq")? {
                Some('r') => {}
                _ => return Ok(false),
            }
            match run.stage() {
                Stage::Connect => *run = Run::new(Params::default()),
                _ => run.restart_rest(),
            }
            step_start = Instant::now();
            continue;
        }
        while let Ok(line) = live.commands.try_recv() {
            let go_on = match run.stage() {
                Stage::Holds => command(run, ui, &line),
                _ => line.trim() != "q",
            };
            if !go_on {
                return Ok(false);
            }
        }
        if run.stage() == Stage::Finished {
            return Ok(true);
        }
        ui.progress(&progress_line(run, waited));
    }
}

/// Steps 1 to 5 on the port. True when a file was written.
fn guided(live: &mut Live, ui: &mut Ui, out: &Path) -> anyhow::Result<bool> {
    let mut run = Run::new(Params::default());
    if !steps(live, &mut run, ui)? {
        ui.line("quit: no calibration file written");
        return Ok(false);
    }
    let (file, text) = report(&run, Some(out))?;
    ui.line(&text);
    if out.exists() {
        let question = format!("replace {}? y + Enter: yes, n + Enter: no", shown(out));
        if live.ask(ui, &question, "yn")? != Some('y') {
            ui.line("kept the current file: no calibration file written");
            return Ok(false);
        }
    }
    ui.line(&write_file(&file, out)?);
    ui.clear();
    println!("    at {}", out.display());
    Ok(true)
}

fn live(port: &PortArgs, out: &Path, dir: &Path) -> anyhow::Result<()> {
    fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;
    let dir = fs::canonicalize(dir)?;
    let now = jiff::Zoned::now();
    let stem = dir.join(format!("{}_calibrate", now.strftime("%Y%m%d-%H%M%S")));
    let started = now.strftime("%Y-%m-%dT%H:%M:%S%:z").to_string();

    let mut serial = serial::open(port)?;
    serial::settle(serial.as_mut())?;
    let bin = stem.with_extension("bin");
    let sink = File::create_new(&bin).with_context(|| format!("creating {}", bin.display()))?;
    let (tx, commands) = mpsc::channel();
    std::thread::spawn(move || {
        for line in io::stdin().lock().lines().map_while(Result::ok) {
            if tx.send(line).is_err() {
                break;
            }
        }
    });
    let mut live = Live {
        serial,
        sink: BufWriter::new(sink),
        deframer: Deframer::new(),
        stats: Stats::new(),
        commands,
    };
    let mut ui = Ui::new();
    // Not in the note: these paths are this machine's.
    println!(
        "calibrating on {}; the run is recorded to {}",
        port.port,
        bin.display()
    );
    let begun = Instant::now();
    let result = guided(&mut live, &mut ui, out);
    ui.clear();
    if let Err(e) = &result {
        ui.log.push_str(&format!("failed: {e:#}\n"));
    }

    // The session is kept whatever happened.
    live.sink.flush()?;
    let session = Session {
        label: "calibrate",
        note: ui.log.trim_end(),
        seconds: begun.elapsed().as_secs(),
        started: &started,
    };
    let toml = stem.with_extension("toml");
    fs::write(&toml, sidecar(&session, &live.stats))
        .with_context(|| format!("writing {}", toml.display()))?;
    println!("session: {} and {}", bin.display(), toml.display());
    if !result? {
        bail!("calibration not finished");
    }
    Ok(())
}

pub fn run(args: &CalibrateArgs) -> anyhow::Result<()> {
    match &args.replay {
        Some(session) => replay(session, args.out.as_deref()),
        None => live(
            &args.port,
            &args.out.clone().unwrap_or_else(default_out),
            &args.dir.clone().unwrap_or_else(default_dir),
        ),
    }
}

#[cfg(test)]
mod tests {
    use protocol::{Firmware, Hello, ImuStatus, Sample, frame};

    use super::*;

    const PERIOD_US: u32 = 9600;
    /// 1 g at +-4 g, LSB.
    const G_LSB: f64 = 1e6 / 122.0;
    const OFFSET_LSB: [f64; 3] = [-35.0, 45.0, 205.0];
    const GAIN: [f64; 3] = [1.009, 0.999, 0.995];
    const BIAS_LSB: [i16; 3] = [104, -324, -14];

    struct Stream {
        bytes: Vec<u8>,
        seq: u16,
        t_us: u32,
    }

    impl Stream {
        fn push(&mut self, m: &Message) {
            self.bytes
                .extend(frame::encode(&m.encode()).expect("frame"));
        }

        /// `seconds` with gravity along `dir` (unit, board axes); `shake` LSB of wobble.
        fn hold(&mut self, dir: [f64; 3], seconds: f64, shake: i16) {
            for _ in 0..(seconds * 1e6 / f64::from(PERIOD_US)) as u32 {
                let w = if self.seq.is_multiple_of(2) {
                    shake
                } else {
                    -shake
                };
                let accel = std::array::from_fn(|i| {
                    (dir[i] * G_LSB / GAIN[i] + OFFSET_LSB[i]).round() as i16 + w
                });
                self.push(&Message::Sample(Sample {
                    seq: self.seq,
                    t_us: self.t_us,
                    gyro: BIAS_LSB.map(|b| b + w),
                    accel,
                }));
                self.seq = self.seq.wrapping_add(1);
                self.t_us = self.t_us.wrapping_add(PERIOD_US);
            }
        }
    }

    /// A whole run as the stream firmware would send it: rest, then the six faces and
    /// eight corners, shaken in between.
    fn session() -> Vec<u8> {
        let mut s = Stream {
            // A stream is joined mid-frame: the deframer syncs on the first delimiter.
            bytes: vec![0],
            seq: 0,
            t_us: u32::MAX - 5_000_000, // the clock wraps during the run
        };
        s.push(&Message::Hello(Hello {
            fw: Firmware::Stream,
            imu: ImuStatus::Ok,
            imu_addr: 0x6B,
            who_am_i: 0x69,
            ctrl1_xl: 0x48,
            ctrl2_g: 0x44,
        }));
        let c = 1.0 / 3f64.sqrt();
        let mut dirs = vec![
            [0.0, 0.0, 1.0],
            [0.0, 0.0, -1.0],
            [1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
        ];
        for k in 0..8 {
            let sign = |bit: u32| if k >> bit & 1 == 0 { c } else { -c };
            dirs.push([sign(0), sign(1), sign(2)]);
        }
        s.hold(dirs[0], 13.0, 2);
        for d in dirs {
            s.hold(d, 0.5, 400);
            s.hold(d, 3.5, 6);
        }
        s.bytes
    }

    #[test]
    fn replay_of_a_whole_run_writes_the_file() {
        let dir = std::env::temp_dir().join(format!("gesture-calibrate-{}", std::process::id()));
        fs::create_dir_all(&dir).expect("dir");
        let (bin, out) = (dir.join("run.bin"), dir.join("board.cal"));
        fs::write(&bin, session()).expect("session");

        replay(&bin, Some(&out)).expect("replay");
        let file = CalFile::parse(&fs::read(&out).expect("file")).expect("valid");
        assert_eq!((file.ctrl1_xl, file.ctrl2_g), (0x48, 0x44));
        for i in 0..3 {
            let offset = f64::from(file.accel_offset_lsb[i]);
            let gain = f64::from(file.accel_gain_q30[i]) / Q30;
            assert!(
                (offset - OFFSET_LSB[i]).abs() <= 2.0,
                "offset {i}: {offset}"
            );
            assert!((gain - GAIN[i]).abs() < 5e-4, "gain {i}: {gain}");
            let bias = f64::from(file.gyro_bias_q8[i]) / Q8;
            assert!(
                (bias - f64::from(BIAS_LSB[i])).abs() < 0.1,
                "bias {i}: {bias}"
            );
        }

        // The second run compares with the first and replaces it.
        replay(&bin, Some(&out)).expect("replay again");
        assert_eq!(CalFile::parse(&fs::read(&out).expect("file")), Ok(file));

        // A run cut short writes nothing.
        let cut = session();
        fs::write(&bin, &cut[..cut.len() / 2]).expect("session");
        fs::remove_file(&out).expect("remove");
        let err = replay(&bin, Some(&out)).expect_err("unfinished");
        assert!(err.to_string().contains("Holds step"), "{err}");
        assert!(!out.exists());
        fs::remove_dir_all(&dir).expect("cleanup");
    }
}
