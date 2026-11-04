//! 3D math for the stroke view, without egui: levelling by gravity, an orbit camera with an
//! orthographic projection, the best-fit plane of a path. f64, for display only.

pub type V3 = [f64; 3];

fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scale(a: V3, k: f64) -> V3 {
    a.map(|x| x * k)
}

fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}

/// `a` with length 1, or `None` if it is (nearly) zero.
fn unit(a: V3) -> Option<V3> {
    let n = norm(a);
    (n > 1e-9).then(|| scale(a, 1.0 / n))
}

/// The part of `a` across the unit vector `n`.
fn across(a: V3, n: V3) -> V3 {
    sub(a, scale(n, dot(a, n)))
}

/// A rotation from a stroke's start frame (the board's axes at the start, `docs/core.md`)
/// to the level frame: z up (along `gravity_ug`), x forward (the board's x made
/// horizontal), y = z x x (left). Rows are the level axes in board coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Level(pub [V3; 3]);

impl Level {
    /// No gravity known: the board frame as it is.
    pub const BOARD: Self = Self([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);

    /// From `gc_stroke_info.gravity_ug` (up in the start frame). Forward is the board's x;
    /// its y when the x axis points within ~75 degrees of vertical (no magnetometer, so
    /// nothing better). Zero gravity gives `BOARD`.
    #[must_use]
    pub fn from_gravity(gravity_ug: [i32; 3]) -> Self {
        let Some(up) = unit(gravity_ug.map(f64::from)) else {
            return Self::BOARD;
        };
        let x = across([1.0, 0.0, 0.0], up);
        let fwd = if norm(x) > 0.25 {
            x
        } else {
            across([0.0, 1.0, 0.0], up)
        };
        let fwd = unit(fwd).expect("x and y cannot both be along up");
        Self([fwd, cross(up, fwd), up])
    }

    #[must_use]
    pub fn apply(&self, p: V3) -> V3 {
        self.0.map(|row| dot(row, p))
    }
}

/// Fixed views. Front looks forward (+x), as the user sees the board while drawing; top
/// looks down with forward up on the screen; side looks left (+y), forward to the right.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Preset {
    Front,
    Top,
    Side,
}

impl Preset {
    pub const ALL: [Self; 3] = [Self::Front, Self::Top, Self::Side];

    pub fn name(self) -> &'static str {
        match self {
            Self::Front => "Front",
            Self::Top => "Top",
            Self::Side => "Side",
        }
    }

    /// The direction the camera looks along.
    fn dir(self) -> V3 {
        match self {
            Self::Front => [1.0, 0.0, 0.0],
            Self::Top => [0.0, 0.0, -1.0],
            Self::Side => [0.0, 1.0, 0.0],
        }
    }
}

/// An orbit camera, orthographic (sizes on the screen do not depend on depth, so the grid
/// reads as a scale). It looks along `(cos p cos y, cos p sin y, -sin p)`: yaw about z,
/// pitch down from horizontal; z stays up on the screen.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Camera {
    pub yaw: f64,
    pub pitch: f64,
    /// The point at the view center, mm.
    pub center: V3,
    pub px_per_mm: f64,
    /// Fit the view to what is drawn on every frame, until the user zooms.
    pub auto_fit: bool,
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            yaw: 0.0,
            pitch: 0.0,
            center: [0.0; 3],
            px_per_mm: 1.0,
            auto_fit: true,
        }
    }
}

/// Fit leaves this much of the half view free around the points.
const FIT_FILL: f64 = 0.9;
/// Fit treats anything smaller as this radius, mm, so a tiny move is not blown up.
const FIT_MIN_RADIUS_MM: f64 = 25.0;
/// Radians per pixel of drag.
const ORBIT_RAD_PER_PX: f64 = 0.01;
/// Zoom range, px per mm: 1 m is 10 px at one end, 1 mm is 50 px at the other.
const PX_PER_MM: std::ops::RangeInclusive<f64> = 0.01..=50.0;

impl Camera {
    /// Screen right, screen up and the view direction (into the screen), in the scene.
    #[must_use]
    pub fn axes(&self) -> [V3; 3] {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        [
            [sy, -cy, 0.0],
            [cy * sp, sy * sp, cp],
            [cp * cy, cp * sy, -sp],
        ]
    }

    /// A scene point -> pixels right of and below the view center, and depth in mm
    /// (larger is farther).
    #[must_use]
    pub fn project(&self, p: V3) -> V3 {
        let [right, up, dir] = self.axes();
        let d = sub(p, self.center);
        [
            dot(d, right) * self.px_per_mm,
            -dot(d, up) * self.px_per_mm,
            dot(d, dir),
        ]
    }

    pub fn look(&mut self, preset: Preset) {
        self.look_along(preset.dir());
    }

    /// Zooms by `factor`, within the zoom range.
    pub fn zoom(&mut self, factor: f64) {
        self.px_per_mm = (self.px_per_mm * factor).clamp(*PX_PER_MM.start(), *PX_PER_MM.end());
    }

    /// Looks along `d`; a vertical `d` keeps the yaw.
    pub fn look_along(&mut self, d: V3) {
        let Some(d) = unit(d) else { return };
        if d[0].hypot(d[1]) > 1e-9 {
            self.yaw = d[1].atan2(d[0]);
        }
        self.pitch = (-d[2]).clamp(-1.0, 1.0).asin();
    }

    /// Drag by pixels: right turns the scene right, down tips it toward the viewer.
    pub fn orbit(&mut self, dx: f64, dy: f64) {
        self.yaw -= dx * ORBIT_RAD_PER_PX;
        self.pitch = (self.pitch + dy * ORBIT_RAD_PER_PX)
            .clamp(-std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2);
    }

    /// Centers on the points' bounding box and scales its bounding sphere to the smaller
    /// view side, so turning the camera does not change the zoom.
    pub fn fit(&mut self, points: &[V3], view_px: f64) {
        let Some(first) = points.first() else { return };
        let (mut lo, mut hi) = (*first, *first);
        for p in points {
            for i in 0..3 {
                lo[i] = lo[i].min(p[i]);
                hi[i] = hi[i].max(p[i]);
            }
        }
        let mid = scale([lo[0] + hi[0], lo[1] + hi[1], lo[2] + hi[2]], 0.5);
        let radius = points
            .iter()
            .map(|p| norm(sub(*p, mid)))
            .fold(FIT_MIN_RADIUS_MM, f64::max);
        self.center = mid;
        self.px_per_mm = 1.0;
        self.zoom(FIT_FILL * view_px / 2.0 / radius);
    }
}

/// Grid steps to choose from, mm.
const GRID_STEPS_MM: [f64; 7] = [10.0, 20.0, 50.0, 100.0, 200.0, 500.0, 1000.0];

/// The smallest grid step at least `min_px` wide on the screen.
#[must_use]
pub fn grid_step_mm(px_per_mm: f64, min_px: f64) -> f64 {
    GRID_STEPS_MM
        .into_iter()
        .find(|s| s * px_per_mm >= min_px)
        .unwrap_or(GRID_STEPS_MM[GRID_STEPS_MM.len() - 1])
}

/// Below this share of the largest spread, a direction counts as flat: two small
/// eigenvalues mean a line, whose plane is not defined.
const LINE_RATIO: f64 = 0.05;

/// Normal of the points' best-fit plane (least spread). For a near-straight path any
/// plane through the line fits; it takes the vertical one (a horizontal normal across the
/// line), so the line is seen from beside it, not from above. `None` below 3 points or for
/// points all in one place.
#[must_use]
pub fn plane_normal(points: &[V3]) -> Option<V3> {
    if points.len() < 3 {
        return None;
    }
    let n = points.len() as f64;
    let mut mean = [0.0; 3];
    for p in points {
        for i in 0..3 {
            mean[i] += p[i] / n;
        }
    }
    let mut cov = [[0.0; 3]; 3];
    for p in points {
        let d = sub(*p, mean);
        for i in 0..3 {
            for j in 0..3 {
                cov[i][j] += d[i] * d[j] / n;
            }
        }
    }
    let (values, vectors) = eigen(cov);
    let mut order = [0, 1, 2];
    order.sort_by(|&a, &b| values[a].total_cmp(&values[b]));
    let col = |k: usize| [vectors[0][k], vectors[1][k], vectors[2][k]];
    let [_, mid, large] = order.map(|k| values[k]);
    if large <= 1e-9 {
        return None;
    }
    if mid > LINE_RATIO * large {
        return Some(col(order[0]));
    }
    // A line: horizontal and across it; for a vertical line, toward forward.
    let line = col(order[2]);
    unit(cross(line, [0.0, 0.0, 1.0])).or_else(|| unit(across([1.0, 0.0, 0.0], line)))
}

/// Which way to look along a plane normal: the sign that is closer to the nearest preset,
/// so the plane view does not mirror from one stroke to the next.
#[must_use]
pub fn plane_view_dir(normal: V3) -> V3 {
    let nearest = Preset::ALL
        .into_iter()
        .map(Preset::dir)
        .max_by(|a, b| dot(normal, *a).abs().total_cmp(&dot(normal, *b).abs()))
        .expect("presets");
    if dot(normal, nearest) < 0.0 {
        scale(normal, -1.0)
    } else {
        normal
    }
}

/// Jacobi eigenvalue iteration for a symmetric 3x3 matrix: the eigenvalues and the
/// eigenvectors as columns.
fn eigen(mut a: [[f64; 3]; 3]) -> (V3, [[f64; 3]; 3]) {
    let mut v = Level::BOARD.0;
    for _ in 0..64 {
        let (p, q) = [(0, 1), (0, 2), (1, 2)]
            .into_iter()
            .max_by(|&(i, j), &(k, l)| a[i][j].abs().total_cmp(&a[k][l].abs()))
            .expect("pairs");
        let size = a[0][0].abs() + a[1][1].abs() + a[2][2].abs();
        if a[p][q].abs() <= 1e-15 * size {
            break;
        }
        let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        let t = theta.signum() / (theta.abs() + theta.mul_add(theta, 1.0).sqrt());
        let c = 1.0 / t.mul_add(t, 1.0).sqrt();
        let s = t * c;
        // A <- J^T A J and V <- V J, J the rotation in the (p, q) plane.
        for row in &mut a {
            let (x, y) = (row[p], row[q]);
            row[p] = c * x - s * y;
            row[q] = s * x + c * y;
        }
        let (rp, rq) = (a[p], a[q]);
        a[p] = std::array::from_fn(|k| c * rp[k] - s * rq[k]);
        a[q] = std::array::from_fn(|k| s * rp[k] + c * rq[k]);
        for row in &mut v {
            let (x, y) = (row[p], row[q]);
            row[p] = c * x - s * y;
            row[q] = s * x + c * y;
        }
    }
    ([a[0][0], a[1][1], a[2][2]], v)
}

#[cfg(test)]
mod tests {
    use std::f64::consts::FRAC_1_SQRT_2;

    use super::*;

    fn near(a: V3, b: V3) -> bool {
        norm(sub(a, b)) < 1e-9
    }

    fn det(m: [V3; 3]) -> f64 {
        dot(m[0], cross(m[1], m[2]))
    }

    fn check_rotation(l: Level) {
        for i in 0..3 {
            for j in 0..3 {
                let want = if i == j { 1.0 } else { 0.0 };
                assert!((dot(l.0[i], l.0[j]) - want).abs() < 1e-9, "{l:?}");
            }
        }
        assert!((det(l.0) - 1.0).abs() < 1e-9, "mirrored: {l:?}");
    }

    #[test]
    fn gravity_up_along_z_is_no_rotation() {
        assert_eq!(Level::from_gravity([0, 0, 1_000_000]), Level::BOARD);
        assert_eq!(Level::from_gravity([0, 0, 0]), Level::BOARD);
    }

    #[test]
    fn levelling_puts_up_up_and_board_x_forward() {
        // Board on its side, y up: y -> up, x stays forward, z -> right (-left).
        let l = Level::from_gravity([0, 1_000_000, 0]);
        check_rotation(l);
        assert!(near(l.apply([0.0, 1.0, 0.0]), [0.0, 0.0, 1.0]));
        assert!(near(l.apply([1.0, 0.0, 0.0]), [1.0, 0.0, 0.0]));
        assert!(near(l.apply([0.0, 0.0, 1.0]), [0.0, -1.0, 0.0]));

        // Nose up by 45 degrees: up has a +x part in the board frame; board x goes to
        // forward-and-up, a move along board z goes to up-and-back.
        let l = Level::from_gravity([707_107, 0, 707_107]);
        check_rotation(l);
        let s = FRAC_1_SQRT_2;
        assert!(near(l.apply([1.0, 0.0, 0.0]), [s, 0.0, s]));
        assert!(near(l.apply([0.0, 0.0, 1.0]), [-s, 0.0, s]));
        assert!(near(l.apply([0.0, 1.0, 0.0]), [0.0, 1.0, 0.0]));

        // Board standing on its x end: forward falls back to its y.
        let l = Level::from_gravity([1_000_000, 0, 0]);
        check_rotation(l);
        assert!(near(l.apply([1.0, 0.0, 0.0]), [0.0, 0.0, 1.0]));
        assert!(near(l.apply([0.0, 1.0, 0.0]), [1.0, 0.0, 0.0]));

        // Any tilt stays a proper rotation with up up.
        let g = [123_000, -456_000, 789_000];
        let l = Level::from_gravity(g);
        check_rotation(l);
        let up = unit(g.map(f64::from)).expect("g");
        assert!(near(l.apply(up), [0.0, 0.0, 1.0]));
    }

    #[test]
    fn presets_project_known_points() {
        let mut cam = Camera {
            px_per_mm: 2.0,
            ..Camera::default()
        };
        // Front: left is screen left, up is screen up (y down), forward is depth.
        cam.look(Preset::Front);
        assert!(near(cam.project([0.0, 100.0, 0.0]), [-200.0, 0.0, 0.0]));
        assert!(near(cam.project([0.0, 0.0, 100.0]), [0.0, -200.0, 0.0]));
        assert!(near(cam.project([100.0, 0.0, 0.0]), [0.0, 0.0, 100.0]));
        // Top: forward is screen up, left is screen left, up is nearer.
        cam.look(Preset::Top);
        assert!(near(cam.project([100.0, 0.0, 0.0]), [0.0, -200.0, 0.0]));
        assert!(near(cam.project([0.0, 100.0, 0.0]), [-200.0, 0.0, 0.0]));
        assert!(near(cam.project([0.0, 0.0, 100.0]), [0.0, 0.0, -100.0]));
        // Side: forward is screen right.
        cam.look(Preset::Side);
        assert!(near(cam.project([100.0, 0.0, 0.0]), [200.0, 0.0, 0.0]));
        assert!(near(cam.project([0.0, 0.0, 100.0]), [0.0, -200.0, 0.0]));
        // The center is at the view center.
        cam.center = [10.0, 20.0, 30.0];
        assert!(near(cam.project([10.0, 20.0, 30.0]), [0.0; 3]));
    }

    #[test]
    fn fit_takes_the_bounding_sphere() {
        let mut cam = Camera::default();
        cam.fit(&[[0.0, 0.0, 0.0], [300.0, 0.0, 0.0]], 600.0);
        assert!(near(cam.center, [150.0, 0.0, 0.0]));
        assert!((cam.px_per_mm - FIT_FILL * 300.0 / 150.0).abs() < 1e-9);
        // The 300 mm line spans FIT_FILL of the view from the front-left diagonal too.
        cam.look_along([1.0, 1.0, 0.0]);
        let ends = [[0.0, 0.0, 0.0], [300.0, 0.0, 0.0]].map(|p| cam.project(p));
        assert!(ends[1][0] - ends[0][0] < FIT_FILL * 600.0);
    }

    #[test]
    fn zoom_stays_in_range() {
        let mut cam = Camera::default();
        cam.fit(&[[0.0; 3], [300.0, 0.0, 0.0]], 0.0);
        assert_eq!(cam.px_per_mm, *PX_PER_MM.start());
        for _ in 0..1000 {
            cam.zoom(2.0);
        }
        assert_eq!(cam.px_per_mm, *PX_PER_MM.end());
    }

    #[test]
    fn plane_of_flat_and_straight_paths() {
        let flat: Vec<V3> = (0..20)
            .map(|i| {
                let a = f64::from(i) * 0.3;
                [100.0 * a.cos(), 50.0 * a.sin(), 0.0]
            })
            .collect();
        let n = plane_normal(&flat).expect("normal");
        assert!(
            near(n, [0.0, 0.0, 1.0]) || near(n, [0.0, 0.0, -1.0]),
            "{n:?}"
        );
        // Seen from the top.
        assert!(near(plane_view_dir(n), [0.0, 0.0, -1.0]));

        // The same drawn on a wall in front of the user (the y-z plane): seen from the front.
        let wall: Vec<V3> = flat.iter().map(|p| [3.0, p[0], p[1]]).collect();
        let d = plane_view_dir(plane_normal(&wall).expect("normal"));
        assert!(near(d, [1.0, 0.0, 0.0]), "{d:?}");

        // A straight line forward with a little wobble: the vertical plane through it,
        // seen from the side.
        let line: Vec<V3> = (0..30)
            .map(|i| {
                let x = f64::from(i) * 10.0;
                [x, (x * 0.1).sin(), (x * 0.07).cos()]
            })
            .collect();
        let d = plane_view_dir(plane_normal(&line).expect("normal"));
        assert!(d[1] > 0.99, "{d:?}");

        assert_eq!(plane_normal(&[[1.0, 2.0, 3.0]; 5]), None);
        assert_eq!(plane_normal(&line[..2]), None);
    }

    #[test]
    fn eigen_diagonalizes() {
        let m = [[4.0, 1.0, 2.0], [1.0, 3.0, 0.5], [2.0, 0.5, 1.0]];
        let (values, v) = eigen(m);
        for k in 0..3 {
            let x = [v[0][k], v[1][k], v[2][k]];
            let mx = m.map(|row| dot(row, x));
            assert!(near(mx, scale(x, values[k])), "{k}");
        }
    }

    #[test]
    fn grid_step_is_wide_enough() {
        assert_eq!(grid_step_mm(1.0, 40.0), 50.0);
        assert_eq!(grid_step_mm(4.0, 40.0), 10.0);
        assert_eq!(grid_step_mm(0.001, 40.0), 1000.0);
    }
}
