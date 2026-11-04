//! The window: a top bar (source, connection, counters) over the stroke list. The state
//! indicator, the event log and the drawing come next (`docs/todo.md` 3e).
//!
//! Elm-shaped: the source's `Update`s fold into `View`; drawing reads the `Model` and
//! returns `Action`s; `Model::act` applies them after the frame. Only the `Form` (what
//! the source should be) is edited by widgets directly.

use std::path::PathBuf;

use egui::{CentralPanel, ComboBox, Panel, RichText, Ui};
use gesture_core::{Event, GcPoint, Kind, Stroke};
use protocol::{Firmware, Hello};
use session::check::Totals;
use session::reference::Setup;
use session::serial::{self, PortArgs};
use session::strokes::Summary;

use crate::feed::Update;
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
    model: Model,
}

/// What the current source has sent.
#[derive(Debug, Default)]
struct View {
    hello: Option<Hello>,
    /// The config line, or why the core could not start.
    config: Option<Result<String, String>>,
    /// The last (re)start reason.
    started: Option<String>,
    samples: u64,
    first_t: Option<u64>,
    last_t: Option<u64>,
    /// The last event that was not `NONE`, at its time.
    last_event: Option<(u64, Event)>,
    /// `gc_live` while moving.
    live: Option<GcPoint>,
    /// Each stroke at its end: time, numbers, corrected points.
    strokes: Vec<(u64, Summary, Stroke)>,
    rejected: u64,
    totals: Totals,
    /// The source stopped: `None` at the end of a replay, else the error.
    ended: Option<Option<String>>,
}

impl View {
    fn apply(&mut self, u: Update) {
        match u {
            Update::Started { why, hello, config } => {
                self.started = Some(why.to_string());
                self.hello = Some(hello);
                self.config = Some(config);
            }
            Update::Sample { t_us, event, live } => {
                self.live = live;
                self.samples += 1;
                self.first_t.get_or_insert(t_us);
                self.last_t = Some(t_us);
                if !event.is_none() {
                    self.last_event = Some((t_us, event));
                }
                if matches!(event.kind, Kind::Rejected(_)) {
                    self.rejected += 1;
                }
            }
            Update::Stroke { stroke, summary } => {
                self.strokes
                    .push((self.last_t.unwrap_or_default(), summary, stroke));
            }
            Update::Health(t) => self.totals = t,
            Update::Ended(e) => self.ended = Some(e),
        }
    }

    /// Seconds since the first sample.
    fn at(&self, t_us: u64) -> f64 {
        t_us.saturating_sub(self.first_t.unwrap_or(t_us)) as f64 / 1e6
    }
}

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
        Self { form, model }
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
        ui.label(format!(
            "{} samples, {rate}, {:.1} s",
            v.samples,
            v.last_t.map_or(0.0, |l| v.at(l))
        ));
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

    fn central(&self, ui: &mut Ui) {
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
        if let Some((t, ev)) = v.last_event {
            ui.label(format!("last event: {} at {:.2} s", ev.kind, v.at(t)));
        }
        if let Some(p) = v.live {
            ui.label(format!(
                "moving: {} {} {} mm from the start (uncorrected)",
                p.x_mm, p.y_mm, p.z_mm
            ));
        }
        ui.separator();
        ui.label(format!(
            "{} strokes, {} rejected",
            v.strokes.len(),
            v.rejected
        ));
        egui::ScrollArea::vertical()
            .stick_to_bottom(true)
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
                        ui.label(format!("{:.2}", v.at(*t)));
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

fn speed_name(speed: f64) -> &'static str {
    SPEEDS
        .iter()
        .find(|(s, _)| *s == speed)
        .map_or("?", |(_, n)| n)
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut Ui, _frame: &mut eframe::Frame) {
        let Self { form, model } = self;
        if let Some(w) = &model.worker {
            for u in w.updates() {
                model.view.apply(u);
            }
        }
        let mut actions = Vec::new();
        Panel::top("source").show(ui, |ui| model.top_bar(form, ui, &mut actions));
        CentralPanel::default().show(ui, |ui| model.central(ui));
        for a in actions {
            model.act(form, a);
        }
    }
}
