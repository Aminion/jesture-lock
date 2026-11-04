//! The window: a top bar (source, connection, counters), a side panel (state, last event,
//! event log), the 3D stroke view (`capture`) and the stroke list below it.
//!
//! Elm-shaped: the source's `Update`s fold into `View`; drawing reads the `Model` and
//! returns `Action`s; `Model::act` applies them after the frame. Only the `Form` (what
//! the source should be) and the `Look` (camera, Keep all) are edited by widgets directly.

use std::collections::VecDeque;
use std::path::PathBuf;

use egui::{CentralPanel, Color32, ComboBox, Panel, RichText, Ui};
use gesture_core::{Event, GcPoint, Kind, Reject, State, Stroke};
use protocol::{Firmware, Hello};
use session::check::Totals;
use session::reference::Setup;
use session::serial::{self, PortArgs};
use session::strokes::Summary;

use crate::capture::{self, Capture, Look};
use crate::feed::{Limits, Update};
use crate::source::{Control, Source, Worker};

/// What to open at start (command line).
#[derive(Debug)]
pub struct Start {
    pub file: Option<PathBuf>,
    pub port: Option<String>,
    pub baud: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Input {
    Replay,
    Serial,
}

const SPEEDS: [(f64, &str); 4] = [
    (1.0, "1x"),
    (2.0, "2x"),
    (5.0, "5x"),
    (f64::INFINITY, "max"),
];

/// What the user asked for in a frame. Widgets only edit the `Form` and push these; the
/// model changes in one place, `Model::act`, after the frame is drawn.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Action {
    /// Start the source the form describes (stops the current one first).
    Connect,
    Stop,
    RefreshPorts,
    /// Replay speed, times real time.
    Speed(f64),
    /// Replay pause.
    Pause(bool),
    /// Wipe the stroke view (the list and the log stay).
    Clear,
}

/// The source being chosen: what the top bar widgets edit directly.
#[derive(Debug)]
struct Form {
    input: Input,
    file: String,
    port: String,
    baud: u32,
}

/// Everything else, changed only by `act` and by the source's updates.
#[derive(Debug)]
struct Model {
    ctx: egui::Context,
    setup: Setup,
    ports: Vec<String>,
    speed: f64,
    paused: bool,
    worker: Option<Worker>,
    view: View,
}

#[derive(Debug)]
pub struct App {
    form: Form,
    look: Look,
    model: Model,
}

/// Event log entries kept; older ones are dropped (a live session runs on).
const LOG_MAX: usize = 1000;

/// How a log line is colored.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Tone {
    Info,
    Good,
    Warn,
    Error,
}

#[derive(Debug, Clone)]
struct Entry {
    /// Seconds on the session clock (`View::now`).
    t: f64,
    tone: Tone,
    text: String,
}

/// What the current source has sent.
#[derive(Debug, Default)]
struct View {
    hello: Option<Hello>,
    /// The config line, or why the core could not start.
    config: Option<Result<String, String>>,
    /// The running core's stroke checks.
    limits: Option<Limits>,
    /// The last (re)start reason.
    started: Option<String>,
    samples: u64,
    /// Session clock: seconds of MCU time since the first sample. A restart resets the MCU
    /// clock (`Feed` unwraps from zero again); this one goes on from where it was.
    now: f64,
    /// MCU time (us) of the first sample since the last start, and `now` at it.
    since_start: Option<(u64, f64)>,
    /// The core's state after the last sample; `None` until the first one.
    state: Option<State>,
    /// The last event that was not `NONE`, at its time.
    last_event: Option<(f64, Event)>,
    /// `gc_live` while moving.
    live: Option<GcPoint>,
    /// Each stroke at its end: time, numbers, corrected points.
    strokes: Vec<(f64, Summary, Stroke)>,
    /// Each move, for the stroke view.
    capture: Capture,
    rejected: u64,
    log: VecDeque<Entry>,
    totals: Totals,
    /// The source stopped: `None` at the end of a replay, else the error.
    ended: Option<Option<String>>,
}

impl View {
    fn apply(&mut self, u: Update) {
        match u {
            Update::Started { why, hello, config } => {
                let (tone, text) = match &config {
                    Ok(_) => (Tone::Info, why.to_string()),
                    Err(e) => (Tone::Error, format!("{why}; core not started: {e}")),
                };
                self.log(tone, text);
                self.started = Some(why.to_string());
                self.hello = Some(hello);
                self.limits = config.as_ref().ok().map(|(_, l)| *l);
                self.config = Some(config.map(|(about, _)| about));
                // The core starts over in WARMUP, on a clock from zero.
                self.state = None;
                self.live = None;
                self.since_start = None;
                self.capture.restart();
            }
            Update::Sample { t_us, event, live } => {
                let (t0, now0) = *self.since_start.get_or_insert((t_us, self.now));
                self.now = now0 + t_us.saturating_sub(t0) as f64 / 1e6;
                self.samples += 1;
                self.state = Some(event.state);
                self.live = live;
                self.capture.sample(&event, live);
                if !event.is_none() {
                    self.last_event = Some((self.now, event));
                    let tone = match event.kind {
                        Kind::Stroke => Tone::Good,
                        Kind::Rejected(_) => Tone::Warn,
                        _ => Tone::Info,
                    };
                    self.log(tone, describe(&event, self.limits));
                }
                if matches!(event.kind, Kind::Rejected(_)) {
                    self.rejected += 1;
                }
            }
            Update::Stroke { stroke, summary } => {
                // Right after its STROKE sample: add the numbers to that log line.
                if let Some(e) = self.log.back_mut()
                    && e.tone == Tone::Good
                {
                    e.text += &format!(
                        ", |d| {:.0} mm, extent {} mm",
                        summary.distance_mm, summary.extent_mm
                    );
                }
                self.capture.stroke(self.strokes.len());
                self.strokes.push((self.now, summary, stroke));
            }
            Update::Health(t) => self.totals = t,
            Update::Ended(e) => {
                match &e {
                    None => self.log(Tone::Info, "end of the session".to_owned()),
                    Some(e) => self.log(Tone::Error, e.clone()),
                }
                self.ended = Some(e);
            }
        }
    }

    fn log(&mut self, tone: Tone, text: String) {
        if self.log.len() == LOG_MAX {
            self.log.pop_front();
        }
        self.log.push_back(Entry {
            t: self.now,
            tone,
            text,
        });
    }
}

/// An event as a log line: the kind, and for a reject why, against the limit it broke
/// (`docs/core.md`, reject reasons).
fn describe(ev: &Event, limits: Option<Limits>) -> String {
    let s = |us: u32| f64::from(us) / 1e6;
    let dur = s(ev.duration_us);
    let detail = match (ev.kind, limits) {
        (Kind::Stroke, _) => format!(
            "{dur:.2} s, v_end {} mm/s, {} points",
            ev.v_end_mm_s, ev.points
        ),
        (Kind::Rejected(Reject::Short), Some(l)) if ev.duration_us < l.min_stroke_us => {
            format!("{dur:.2} s < {:.2} s", s(l.min_stroke_us))
        }
        // Duration passed, so the extent did not (it is checked last).
        (Kind::Rejected(Reject::Short), Some(l)) => {
            format!("extent < {} mm, {dur:.2} s", l.min_extent_mm)
        }
        (Kind::Rejected(Reject::Drift), Some(l)) => format!(
            "v_end {} > {} mm/s, {dur:.2} s",
            ev.v_end_mm_s, l.max_v_end_mm_s
        ),
        (Kind::Rejected(Reject::Long), Some(l)) => {
            format!("longer than {:.2} s", s(l.max_stroke_us))
        }
        (Kind::Rejected(Reject::Clipped), _) => {
            format!("a clipped reading during the move, {dur:.2} s")
        }
        (Kind::Rejected(Reject::TimeGap), _) => "a time gap during the move".to_owned(),
        (Kind::Rejected(r), None) if r.at_end_hold() => {
            format!("{dur:.2} s, v_end {} mm/s", ev.v_end_mm_s)
        }
        _ => String::new(),
    };
    if detail.is_empty() {
        ev.kind.to_string()
    } else {
        format!("{}: {detail}", ev.kind)
    }
}

const GREEN: Color32 = Color32::from_rgb(70, 170, 80);
pub const BLUE: Color32 = Color32::from_rgb(70, 140, 230);

impl App {
    pub fn new(ctx: egui::Context, setup: Setup, start: Start) -> Self {
        let ports = serial::ports();
        let form = Form {
            input: if start.port.is_some() {
                Input::Serial
            } else {
                Input::Replay
            },
            file: start
                .file
                .as_ref()
                .map(|f| f.display().to_string())
                .unwrap_or_default(),
            port: start
                .port
                .clone()
                .or_else(|| ports.first().cloned())
                .unwrap_or_default(),
            baud: start.baud,
        };
        let mut model = Model {
            ctx,
            setup,
            ports,
            speed: 1.0,
            paused: false,
            worker: None,
            view: View::default(),
        };
        if start.file.is_some() || start.port.is_some() {
            model.act(&form, Action::Connect);
        }
        Self {
            form,
            look: Look::default(),
            model,
        }
    }
}

impl Model {
    fn act(&mut self, form: &Form, action: Action) {
        match action {
            Action::Connect => {
                self.worker = None;
                let source = match form.input {
                    Input::Replay => Source::Replay(PathBuf::from(form.file.trim())),
                    Input::Serial => Source::Serial(PortArgs {
                        port: form.port.clone(),
                        baud: form.baud,
                    }),
                };
                self.paused = false;
                self.view = View::default();
                let ctx = self.ctx.clone();
                self.worker = Some(Worker::start(source, self.setup, self.speed, move || {
                    ctx.request_repaint();
                }));
            }
            Action::Stop => self.worker = None,
            Action::RefreshPorts => self.ports = serial::ports(),
            Action::Speed(s) => {
                self.speed = s;
                self.send(Control::Speed(s));
            }
            Action::Pause(p) => {
                self.paused = p;
                self.send(Control::Pause(p));
            }
            Action::Clear => self.view.capture.clear(),
        }
    }

    fn send(&self, c: Control) {
        if let Some(w) = &self.worker {
            w.send(c);
        }
    }

    fn running(&self) -> bool {
        self.worker.is_some() && self.view.ended.is_none()
    }

    fn top_bar(&self, form: &mut Form, ui: &mut Ui, actions: &mut Vec<Action>) {
        ui.horizontal_wrapped(|ui| {
            let running = self.running();
            ui.add_enabled_ui(!running, |ui| {
                ui.selectable_value(&mut form.input, Input::Replay, "Replay");
                ui.selectable_value(&mut form.input, Input::Serial, "Serial");
                ui.separator();
                match form.input {
                    Input::Replay => {
                        ui.label("file");
                        ui.text_edit_singleline(&mut form.file);
                    }
                    Input::Serial => {
                        ComboBox::from_id_salt("port")
                            .selected_text(&form.port)
                            .show_ui(ui, |ui| {
                                for p in &self.ports {
                                    ui.selectable_value(&mut form.port, p.clone(), p);
                                }
                            });
                        if ui.button("Refresh").clicked() {
                            actions.push(Action::RefreshPorts);
                        }
                    }
                }
            });
            if running {
                if ui.button("Stop").clicked() {
                    actions.push(Action::Stop);
                }
            } else if ui.button("Start").clicked() {
                actions.push(Action::Connect);
            }
            if form.input == Input::Replay {
                ui.separator();
                let mut speed = self.speed;
                ComboBox::from_id_salt("speed")
                    .selected_text(speed_name(speed))
                    .show_ui(ui, |ui| {
                        for (s, name) in SPEEDS {
                            ui.selectable_value(&mut speed, s, name);
                        }
                    });
                if speed != self.speed {
                    actions.push(Action::Speed(speed));
                }
                if running
                    && ui
                        .button(if self.paused { "Resume" } else { "Pause" })
                        .clicked()
                {
                    actions.push(Action::Pause(!self.paused));
                }
            }
        });
        ui.horizontal_wrapped(|ui| self.counters(ui));
    }

    fn counters(&self, ui: &mut Ui) {
        let v = &self.view;
        let t = &v.totals;
        let rate = t.rate_hz().map_or("-".to_owned(), |r| format!("{r:.1} Hz"));
        ui.label(format!("{} samples, {rate}, {:.1} s", v.samples, v.now));
        let errors = [
            ("frame errors", t.frame_errors.total()),
            ("message errors", t.message_errors),
            ("lost", t.lost),
            ("t gaps", t.t_gaps),
            ("resets", t.resets),
            ("mcu missed", t.mcu.missed),
            ("stuck", t.mcu.stuck),
            ("i2c", t.mcu.i2c_err),
        ];
        for (name, n) in errors {
            let text = RichText::new(format!("{name} {n}"));
            ui.label(if n > 0 {
                text.color(ui.visuals().warn_fg_color)
            } else {
                text
            });
        }
    }

    fn central(&self, ui: &mut Ui, look: &mut Look, actions: &mut Vec<Action>) {
        let v = &self.view;
        match &v.ended {
            Some(None) => {
                ui.label("End of the session.");
            }
            Some(Some(e)) => {
                ui.colored_label(ui.visuals().error_fg_color, e);
            }
            None if self.worker.is_none() => {
                ui.label("Not connected.");
            }
            None if v.hello.is_none() => {
                ui.label("Waiting for Hello...");
            }
            None => {}
        }
        if let Some(h) = v.hello {
            ui.label(format!(
                "{:?} firmware, IMU {:?} (WHO_AM_I {:#04x}){}",
                h.fw,
                h.imu,
                h.who_am_i,
                v.started
                    .as_ref()
                    .map_or(String::new(), |s| format!(", {s}"))
            ));
            if h.fw != Firmware::Stream {
                ui.colored_label(
                    ui.visuals().warn_fg_color,
                    "Not the stream firmware: it sends no samples. Flash `stream`.",
                );
            }
        }
        match &v.config {
            Some(Ok(about)) => {
                ui.label(about);
            }
            Some(Err(e)) => {
                ui.colored_label(
                    ui.visuals().error_fg_color,
                    format!("core not started: {e}"),
                );
            }
            None => {}
        }
        ui.separator();
        if capture::show(ui, look, &v.capture, &v.strokes) {
            actions.push(Action::Clear);
        }
    }

    fn strokes_table(&self, ui: &mut Ui) {
        let v = &self.view;
        ui.label(format!(
            "{} strokes, {} rejected",
            v.strokes.len(),
            v.rejected
        ));
        egui::ScrollArea::vertical()
            .stick_to_bottom(true)
            .auto_shrink(false)
            .show(ui, |ui| {
                egui::Grid::new("strokes").striped(true).show(ui, |ui| {
                    for h in [
                        "#",
                        "t, s",
                        "dur, s",
                        "|d|, mm",
                        "end x/y/z, mm",
                        "peak m/s",
                        "v_end mm/s",
                        "extent",
                        "points",
                    ] {
                        ui.strong(h);
                    }
                    ui.end_row();
                    for (i, (t, s, stroke)) in v.strokes.iter().enumerate() {
                        ui.label(format!("{}", i + 1));
                        ui.label(format!("{t:.2}"));
                        ui.label(format!("{:.2}", f64::from(s.duration_us) / 1e6));
                        ui.label(format!("{:.1}", s.distance_mm));
                        ui.label(format!("{} {} {}", s.end_mm[0], s.end_mm[1], s.end_mm[2]));
                        ui.label(format!("{:.2}", s.peak_mm_s / 1e3));
                        ui.label(format!("{}", s.v_end_mm_s));
                        ui.label(format!("{}", s.extent_mm));
                        ui.label(format!("{} (step {})", stroke.points.len(), s.step));
                        ui.end_row();
                    }
                });
            });
    }
}

impl Model {
    /// The state as a word and a color, and what it means for the user.
    fn state(&self) -> (&'static str, Color32, &'static str) {
        let v = &self.view;
        let grey = Color32::GRAY;
        if v.ended.is_some() {
            return ("STOPPED", grey, "The source stopped.");
        }
        if self.worker.is_none() {
            return ("OFF", grey, "Not connected.");
        }
        if let Some(Err(_)) = v.config {
            return ("NO CORE", Color32::RED, "The core could not be configured.");
        }
        match v.state {
            None => ("WAITING", grey, "No samples yet."),
            Some(State::Warmup) => ("WARMUP", Color32::from_rgb(220, 160, 40), "Hold still."),
            Some(State::Still) => ("READY", GREEN, "Holding; move to draw a stroke."),
            Some(State::Moving) => ("MOVING", BLUE, "Stop and hold to end the stroke."),
        }
    }

    fn side(&self, ui: &mut Ui) {
        let v = &self.view;
        let (word, color, hint) = self.state();
        ui.label(RichText::new(word).size(28.0).strong().color(color));
        ui.label(hint);
        ui.separator();
        match v.last_event {
            Some((t, ev)) => {
                ui.label(format!("last event at {t:.2} s"));
                ui.label(RichText::new(describe(&ev, v.limits)).color(tone_color(
                    ui,
                    match ev.kind {
                        Kind::Stroke => Tone::Good,
                        Kind::Rejected(_) => Tone::Warn,
                        _ => Tone::Info,
                    },
                )));
            }
            None => {
                ui.label("no events yet");
            }
        }
        if let Some(p) = v.live {
            ui.label(format!(
                "moving: {} {} {} mm from the start (uncorrected)",
                p.x_mm, p.y_mm, p.z_mm
            ));
        }
        ui.separator();
        ui.strong("Event log");
        egui::ScrollArea::vertical()
            .stick_to_bottom(true)
            .auto_shrink(false)
            .show(ui, |ui| {
                for e in &v.log {
                    ui.label(
                        RichText::new(format!("{:8.2}  {}", e.t, e.text))
                            .monospace()
                            .color(tone_color(ui, e.tone)),
                    );
                }
            });
    }
}

fn tone_color(ui: &Ui, tone: Tone) -> Color32 {
    match tone {
        Tone::Info => ui.visuals().text_color(),
        Tone::Good => GREEN,
        Tone::Warn => ui.visuals().warn_fg_color,
        Tone::Error => ui.visuals().error_fg_color,
    }
}

fn speed_name(speed: f64) -> &'static str {
    SPEEDS
        .iter()
        .find(|(s, _)| *s == speed)
        .map_or("?", |(_, n)| n)
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut Ui, _frame: &mut eframe::Frame) {
        let Self { form, look, model } = self;
        if let Some(w) = &model.worker {
            for u in w.updates() {
                model.view.apply(u);
            }
        }
        let mut actions = Vec::new();
        Panel::top("source").show(ui, |ui| model.top_bar(form, ui, &mut actions));
        Panel::right("events")
            .resizable(true)
            .default_size(380.0)
            .show(ui, |ui| model.side(ui));
        Panel::bottom("strokes")
            .resizable(true)
            .default_size(150.0)
            .show(ui, |ui| model.strokes_table(ui));
        CentralPanel::default().show(ui, |ui| model.central(ui, look, &mut actions));
        for a in actions {
            model.act(form, a);
        }
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use session::reference::CoreArgs;

    use super::*;
    use crate::source::replay_updates;

    /// `docs/todo.md` 3e.3: ruler-30 replayed into the view gives the events of
    /// `core/contract/vectors/v1/README.md` in the log, each reject with its reason, and the
    /// state follows every sample (it ends READY after the last hold).
    #[test]
    fn ruler_30_log_and_state() {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../data/sessions");
        let setup = Setup::load(&CoreArgs::vectors(&dir)).expect("setup");
        let mut view = View::default();
        let mut moved = false;
        for u in replay_updates(&dir.join("20261030-211435_ruler-30.bin"), setup) {
            view.apply(u);
            moved |= view.state == Some(State::Moving);
        }
        assert!(moved);
        assert_eq!(view.state, Some(State::Still));
        assert!(view.limits.is_some());

        let count = |p: &str| view.log.iter().filter(|e| e.text.starts_with(p)).count();
        assert_eq!(count("STROKE"), 4);
        assert_eq!(count("REJ SHORT: "), 3);
        assert_eq!(count("REJ DRIFT: v_end "), 1);
        assert_eq!(view.strokes.len(), 4);
        // Each stroke line got its summary.
        assert!(
            view.log
                .iter()
                .filter(|e| e.text.starts_with("STROKE"))
                .all(|e| e.text.contains("|d| "))
        );
        // The session clock only goes forward.
        assert!(
            view.log
                .iter()
                .zip(view.log.iter().skip(1))
                .all(|(a, b)| a.t <= b.t)
        );
    }

    /// `docs/todo.md` 3e.4: ruler-30 replayed into the view gives a shot per move. Each
    /// accepted one keeps its live path; levelled, its points are a straight, level line of
    /// about 30 cm (the ruler lay on a table). Each reject ends its shot greyed.
    #[test]
    fn ruler_30_shots() {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../data/sessions");
        let setup = Setup::load(&CoreArgs::vectors(&dir)).expect("setup");
        let mut view = View::default();
        let (mut strokes, mut rejects, mut long) = (0, 0, 0);
        for u in replay_updates(&dir.join("20261030-211435_ruler-30.bin"), setup) {
            let stroke = matches!(u, Update::Stroke { .. });
            let reject = matches!(u, Update::Sample { event, .. } if matches!(event.kind, Kind::Rejected(_)));
            view.apply(u);
            if !(stroke || reject) {
                continue;
            }
            let shot = view.capture.current().expect("a shot");
            if reject {
                rejects += 1;
                assert!(matches!(shot.end, crate::capture::End::Rejected(_)));
            }
            if !stroke {
                continue;
            }
            strokes += 1;
            let crate::capture::End::Stroke(i) = shot.end else {
                panic!("{:?}", shot.end);
            };
            assert!(!shot.live.is_empty());
            let s = &view.strokes[i].2;
            let (_, p) = crate::capture::levelled(s);
            let end = *p.last().expect("points");
            let len = end[0].hypot(end[1]).hypot(end[2]);
            // Largest distance from the start-end line.
            let off = p
                .iter()
                .map(|q| {
                    let t = (q[0] * end[0] + q[1] * end[1] + q[2] * end[2]) / (len * len);
                    (0..3)
                        .map(|k| (q[k] - t * end[k]).powi(2))
                        .sum::<f64>()
                        .sqrt()
                })
                .fold(0.0, f64::max);
            // The board lay flat: up is the board's z.
            assert!(s.info.gravity_ug[2] > 990_000, "{:?}", s.info.gravity_ug);
            if len > 100.0 {
                long += 1;
                assert!((200.0..400.0).contains(&len), "{len}");
                assert!(off < 0.02 * len, "{off} off a {len} mm line");
                assert!(end[2].abs() < 0.03 * len, "{end:?}");
            }
        }
        assert_eq!((strokes, rejects), (4, 4));
        // The first stroke is a 25 mm nudge; the other three slide the ruler's length.
        assert_eq!(long, 3);
    }
}
