//! Stroke capture (`docs/todo.md` 3e.4): each move as a "shot" (its live path, then how it
//! ended), and the 3D view that draws them on the egui `Painter` through `scene`.

use egui::{Align2, Color32, FontId, Pos2, Response, Sense, Ui, pos2, vec2};
use gesture_core::{Event, GcPoint, Kind, Reject, Stroke};
use session::strokes::Summary;

use crate::app::BLUE;
use crate::scene::{self, Camera, Level, Preset, V3};

/// How a move ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum End {
    Moving,
    /// Index into the view's strokes.
    Stroke(usize),
    Rejected(Reject),
}

/// One move: `gc_live` at every sample from `MOVING` on, in the board frame at its start
/// (uncorrected; it runs on through the end hold, past the stroke's last point).
#[derive(Debug, Clone)]
pub struct Shot {
    pub live: Vec<V3>,
    pub end: End,
}

/// The moves of a session. Rejected moves and the live paths of older strokes are dropped
/// when the next move starts; accepted ones stay for Keep all.
#[derive(Debug, Default)]
pub struct Capture {
    shots: Vec<Shot>,
    /// Shots before this were cleared from the view.
    shown_from: usize,
}

fn mm(p: &GcPoint) -> V3 {
    [p.x_mm, p.y_mm, p.z_mm].map(f64::from)
}

impl Capture {
    /// A sample's event and `gc_live`.
    pub fn sample(&mut self, event: &Event, live: Option<GcPoint>) {
        match event.kind {
            Kind::Moving => self.begin(),
            Kind::Rejected(r) => {
                if let Some(shot) = self.open() {
                    shot.end = End::Rejected(r);
                }
            }
            _ => {}
        }
        if let (Some(p), Some(shot)) = (live, self.open()) {
            shot.live.push(mm(&p));
        }
    }

    /// `Update::Stroke`, right after its `STROKE` sample: the move became stroke `index`.
    pub fn stroke(&mut self, index: usize) {
        match self.open() {
            Some(shot) => shot.end = End::Stroke(index),
            // Not seen from its start (cannot happen with a core started with the stream).
            None => self.shots.push(Shot {
                live: Vec::new(),
                end: End::Stroke(index),
            }),
        }
    }

    /// The core started over: a move in progress will not end.
    pub fn restart(&mut self) {
        if self.open().is_some() {
            self.shots.pop();
        }
        self.shown_from = self.shown_from.min(self.shots.len());
    }

    /// Clear: nothing shown until the next move.
    pub fn clear(&mut self) {
        self.shown_from = self.shots.len();
    }

    fn begin(&mut self) {
        match self.shots.last_mut() {
            Some(Shot {
                end: End::Moving | End::Rejected(_),
                ..
            }) => {
                self.shots.pop();
            }
            // Only the last stroke shows its live path.
            Some(last) => last.live = Vec::new(),
            None => {}
        }
        self.shown_from = self.shown_from.min(self.shots.len());
        self.shots.push(Shot {
            live: Vec::new(),
            end: End::Moving,
        });
    }

    fn open(&mut self) -> Option<&mut Shot> {
        self.shots.last_mut().filter(|s| s.end == End::Moving)
    }

    /// The last shot not cleared.
    #[must_use]
    pub fn current(&self) -> Option<&Shot> {
        self.shots[self.shown_from..].last()
    }

    /// Strokes before the current shot, not cleared.
    fn kept(&self) -> impl Iterator<Item = usize> + '_ {
        let shown = &self.shots[self.shown_from..];
        shown[..shown.len().saturating_sub(1)]
            .iter()
            .filter_map(|s| match s.end {
                End::Stroke(i) => Some(i),
                _ => None,
            })
    }
}

/// What the user looks at; edited by the widgets directly, like the `Form`.
#[derive(Debug, Default)]
pub struct Look {
    pub camera: Camera,
    pub keep_all: bool,
}

/// A path to draw, in the scene (mm).
struct Path {
    points: Vec<V3>,
    color: Color32,
    width: f32,
    /// Start and end markers.
    ends: bool,
}

/// A stroke's rotation by its gravity, and its points in the level frame.
pub fn levelled(stroke: &Stroke) -> (Level, Vec<V3>) {
    let level = Level::from_gravity(stroke.info.gravity_ug);
    (
        level,
        stroke.points.iter().map(|p| level.apply(mm(p))).collect(),
    )
}

/// The paths of what is shown, and whether the current one is levelled.
fn paths(
    capture: &Capture,
    strokes: &[(f64, Summary, Stroke)],
    keep_all: bool,
) -> (Vec<Path>, bool) {
    let mut out = Vec::new();
    if keep_all {
        for i in capture.kept() {
            out.push(Path {
                points: levelled(&strokes[i].2).1,
                color: BLUE.gamma_multiply(0.4),
                width: 1.5,
                ends: false,
            });
        }
    }
    let Some(shot) = capture.current() else {
        return (out, false);
    };
    let board = || shot.live.clone();
    match shot.end {
        End::Moving => out.push(Path {
            points: board(),
            color: BLUE,
            width: 2.0,
            ends: true,
        }),
        End::Rejected(_) => out.push(Path {
            points: board(),
            color: Color32::GRAY,
            width: 1.5,
            ends: true,
        }),
        End::Stroke(i) => {
            let (level, points) = levelled(&strokes[i].2);
            // The same rotation for both, so the gap between them is the drift.
            out.push(Path {
                points: shot.live.iter().map(|p| level.apply(*p)).collect(),
                color: Color32::GRAY.gamma_multiply(0.6),
                width: 1.0,
                ends: false,
            });
            out.push(Path {
                points,
                color: BLUE,
                width: 2.5,
                ends: true,
            });
            return (out, true);
        }
    }
    (out, false)
}

/// The numbers of the current shot.
fn caption(capture: &Capture, strokes: &[(f64, Summary, Stroke)]) -> String {
    let Some(shot) = capture.current() else {
        return "Move to draw a stroke.".to_owned();
    };
    match shot.end {
        End::Moving => match shot.live.last() {
            Some(p) => format!(
                "moving: {:.0} {:.0} {:.0} mm from the start (uncorrected)",
                p[0], p[1], p[2]
            ),
            None => "moving".to_owned(),
        },
        End::Rejected(r) => format!("{} (the move, uncorrected)", Kind::Rejected(r)),
        End::Stroke(i) => {
            let s = &strokes[i].1;
            format!(
                "stroke {}: |d| {:.0} mm, {:.2} s, peak {:.2} m/s, v_end {} mm/s, extent {} mm",
                i + 1,
                s.distance_mm,
                f64::from(s.duration_us) / 1e6,
                s.peak_mm_s / 1e3,
                s.v_end_mm_s,
                s.extent_mm
            )
        }
    }
}

/// Pixels a grid step is at least wide.
const GRID_MIN_PX: f64 = 40.0;
/// Zoom per scrolled point.
const ZOOM_PER_SCROLL: f64 = 0.002;
/// Grid lines each way from the center, at most.
const GRID_MAX_LINES: f64 = 100.0;
/// Axis marker arm, pixels.
const AXIS_PX: f64 = 28.0;

/// The toolbar and the canvas. Returns true when Clear was pressed.
pub fn show(
    ui: &mut Ui,
    look: &mut Look,
    capture: &Capture,
    strokes: &[(f64, Summary, Stroke)],
) -> bool {
    let (paths, level) = paths(capture, strokes, look.keep_all);
    let mut clear = false;
    ui.horizontal_wrapped(|ui| {
        for p in Preset::ALL {
            if ui.button(p.name()).clicked() {
                look.camera.look(p);
            }
        }
        let main = paths
            .last()
            .map(|p| p.points.as_slice())
            .unwrap_or_default();
        let normal = scene::plane_normal(main);
        if ui
            .add_enabled(normal.is_some(), egui::Button::new("Plane"))
            .on_hover_text("Look along the normal of the stroke's best-fit plane")
            .clicked()
            && let Some(n) = normal
        {
            look.camera.look_along(scene::plane_view_dir(n));
        }
        if ui
            .selectable_label(look.camera.auto_fit, "Fit")
            .on_hover_text("Keep everything in view (scrolling turns it off)")
            .clicked()
        {
            look.camera.auto_fit = !look.camera.auto_fit;
        }
        ui.separator();
        ui.checkbox(&mut look.keep_all, "Keep all");
        clear = ui.button("Clear").clicked();
    });
    ui.label(caption(capture, strokes));
    ui.weak(if level {
        "level frame: x forward (the board's x at the start), y left, z up"
    } else {
        "board axes at the move start (levelled when the stroke is accepted)"
    });

    let (response, painter) = ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
    let rect = response.rect;
    if rect.width() < 1.0 || rect.height() < 1.0 {
        return clear;
    }
    let painter = painter.with_clip_rect(rect);
    painter.rect_filled(rect, 0.0, ui.visuals().extreme_bg_color);
    navigate(ui, &response, &mut look.camera);

    let all: Vec<V3> = paths
        .iter()
        .flat_map(|p| p.points.iter().copied())
        .collect();
    let cam = &mut look.camera;
    if cam.auto_fit {
        cam.fit(&all, f64::from(rect.width().min(rect.height())));
    }
    let cam = *cam;
    let to_screen = |p: V3| {
        let [x, y, depth] = cam.project(p);
        (rect.center() + vec2(x as f32, y as f32), depth)
    };
    let faint = ui.visuals().weak_text_color().gamma_multiply(0.35);
    let step = scene::grid_step_mm(cam.px_per_mm, GRID_MIN_PX);
    if !all.is_empty() {
        grid(
            &painter,
            &to_screen,
            &cam,
            &all,
            rect.size().max_elem(),
            step,
            faint,
        );
    }
    draw_paths(&painter, &to_screen, &paths);
    axes(
        &painter,
        &cam,
        rect.left_bottom() + vec2(40.0, -40.0),
        level,
    );
    scale_bar(
        ui,
        &painter,
        &cam,
        rect.right_bottom() + vec2(-20.0, -20.0),
        step,
    );
    clear
}

/// Drag orbits, scroll zooms (and stops the fit), double click fits again.
fn navigate(ui: &Ui, response: &Response, cam: &mut Camera) {
    if response.dragged() {
        let d = response.drag_delta();
        cam.orbit(f64::from(d.x), f64::from(d.y));
    }
    if response.hovered() {
        let scroll = f64::from(ui.input(|i| i.smooth_scroll_delta.y));
        if scroll != 0.0 {
            cam.zoom((scroll * ZOOM_PER_SCROLL).exp());
            cam.auto_fit = false;
        }
    }
    if response.double_clicked() {
        cam.auto_fit = true;
    }
}

/// The floor under the lowest point: a grid of `step` around the view center.
fn grid(
    painter: &egui::Painter,
    to_screen: &impl Fn(V3) -> (Pos2, f64),
    cam: &Camera,
    points: &[V3],
    view_px: f32,
    step: f64,
    color: Color32,
) {
    let floor = points.iter().map(|p| p[2]).fold(f64::INFINITY, f64::min);
    let half = (f64::from(view_px) / cam.px_per_mm / step)
        .ceil()
        .min(GRID_MAX_LINES);
    let snap = |v: f64| (v / step).round() * step;
    let (cx, cy) = (snap(cam.center[0]), snap(cam.center[1]));
    let r = half * step;
    let line = |a: V3, b: V3| {
        painter.line_segment(
            [to_screen(a).0, to_screen(b).0],
            egui::Stroke::new(1.0, color),
        );
    };
    let mut k = -half;
    while k <= half {
        let o = k * step;
        line([cx + o, cy - r, floor], [cx + o, cy + r, floor]);
        line([cx - r, cy + o, floor], [cx + r, cy + o, floor]);
        k += 1.0;
    }
}

enum Item {
    Segment([Pos2; 2], egui::Stroke),
    Dot(Pos2, Color32, bool),
}

/// Every segment and marker, far ones first (no depth buffer).
fn draw_paths(painter: &egui::Painter, to_screen: &impl Fn(V3) -> (Pos2, f64), paths: &[Path]) {
    let mut items: Vec<(f64, Item)> = Vec::new();
    for path in paths {
        let screen: Vec<_> = path.points.iter().map(|p| to_screen(*p)).collect();
        let stroke = egui::Stroke::new(path.width, path.color);
        for w in screen.windows(2) {
            let ((a, da), (b, db)) = (w[0], w[1]);
            items.push(((da + db) / 2.0, Item::Segment([a, b], stroke)));
        }
        if path.ends
            && let (Some(&(a, da)), Some(&(b, db))) = (screen.first(), screen.last())
        {
            items.push((da, Item::Dot(a, path.color, false)));
            items.push((db, Item::Dot(b, path.color, true)));
        }
    }
    items.sort_by(|a, b| b.0.total_cmp(&a.0));
    for (_, item) in items {
        match item {
            Item::Segment(p, s) => {
                painter.line_segment(p, s);
            }
            // Start hollow, end filled.
            Item::Dot(p, c, true) => {
                painter.circle_filled(p, 4.5, c);
            }
            Item::Dot(p, c, false) => {
                painter.circle_stroke(p, 5.0, egui::Stroke::new(1.5, c));
            }
        }
    }
}

/// Which way the scene axes point on the screen.
fn axes(painter: &egui::Painter, cam: &Camera, origin: Pos2, level: bool) {
    let [right, up, _] = cam.axes();
    let names = if level {
        ["fwd", "left", "up"]
    } else {
        ["x", "y", "z"]
    };
    let colors = [
        Color32::from_rgb(220, 70, 70),
        Color32::from_rgb(70, 180, 70),
        Color32::from_rgb(80, 120, 240),
    ];
    for (i, (name, color)) in names.into_iter().zip(colors).enumerate() {
        let (r, u) = (right[i], up[i]);
        let tip = origin + vec2((r * AXIS_PX) as f32, (-u * AXIS_PX) as f32);
        painter.line_segment([origin, tip], egui::Stroke::new(2.0, color));
        let label = origin + (tip - origin) * 1.35;
        painter.text(
            label,
            Align2::CENTER_CENTER,
            name,
            FontId::proportional(11.0),
            color,
        );
    }
}

/// A bar of one grid step, so lengths read off the screen (orthographic).
fn scale_bar(ui: &Ui, painter: &egui::Painter, cam: &Camera, right_end: Pos2, step: f64) {
    let color = ui.visuals().text_color();
    let len = (step * cam.px_per_mm) as f32;
    let left = right_end - vec2(len, 0.0);
    let s = egui::Stroke::new(1.5, color);
    painter.line_segment([left, right_end], s);
    for x in [left, right_end] {
        painter.line_segment([x - vec2(0.0, 4.0), x + vec2(0.0, 4.0)], s);
    }
    let text = if step >= 1000.0 {
        format!("{} m", step / 1000.0)
    } else {
        format!("{step} mm")
    };
    painter.text(
        pos2((left.x + right_end.x) / 2.0, right_end.y - 6.0),
        Align2::CENTER_BOTTOM,
        text,
        FontId::proportional(12.0),
        color,
    );
}

#[cfg(test)]
mod tests {
    use gesture_core::State;

    use super::*;

    fn ev(kind: Kind) -> Event {
        Event {
            kind,
            state: State::Moving,
            ..Event::default()
        }
    }

    fn live(x: i16) -> Option<GcPoint> {
        Some(GcPoint {
            x_mm: x,
            ..GcPoint::default()
        })
    }

    #[test]
    fn shots_keep_strokes_drop_rejects_and_clear() {
        let mut c = Capture::default();
        assert!(c.current().is_none());
        c.sample(&ev(Kind::Moving), live(1));
        c.sample(&ev(Kind::Rejected(Reject::Short)), None);
        assert_eq!(
            c.current().map(|s| s.end),
            Some(End::Rejected(Reject::Short))
        );
        // The reject goes when the next move starts.
        c.sample(&ev(Kind::Moving), live(2));
        c.sample(&ev(Kind::None), live(3));
        c.sample(&ev(Kind::Stroke), None);
        c.stroke(0);
        assert_eq!(
            c.current().map(|s| s.live.clone()),
            Some(vec![[2.0, 0.0, 0.0], [3.0, 0.0, 0.0]])
        );
        c.sample(&ev(Kind::Moving), live(4));
        c.sample(&ev(Kind::Stroke), None);
        c.stroke(1);
        assert_eq!(c.shots.len(), 2);
        assert_eq!(c.current().map(|s| s.end), Some(End::Stroke(1)));
        assert_eq!(c.kept().collect::<Vec<_>>(), [0]);
        // An older stroke drops its live path.
        assert!(c.shots[0].live.is_empty());

        c.clear();
        assert!(c.current().is_none());
        assert_eq!(c.kept().count(), 0);
        c.sample(&ev(Kind::Moving), live(5));
        assert_eq!(c.current().map(|s| s.end), Some(End::Moving));
        assert_eq!(c.kept().count(), 0);
        // A restart drops the move in progress; the view stays cleared.
        c.restart();
        assert!(c.current().is_none());
        c.sample(&ev(Kind::Moving), live(6));
        c.sample(&ev(Kind::Rejected(Reject::Drift)), None);
        c.sample(&ev(Kind::Moving), live(7));
        assert_eq!(
            c.current().map(|s| s.live.clone()),
            Some(vec![[7.0, 0.0, 0.0]])
        );
        assert_eq!(c.shots.len(), 3);
    }
}
