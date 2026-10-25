//! Tracé : trajectoire planifiée (spline fermée), points de passage et sillage du drone.

use std::collections::VecDeque;

use bevy::prelude::*;

use crate::sim::Flight;
use crate::theme;
use dragonskytriker_core::FlightMode;

/// Sillage, points de passage et chemin planifié.
pub struct PathPlugin;

impl Plugin for PathPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Trail>()
            .add_systems(Startup, configure_gizmos)
            .add_systems(Update, (record_trail, draw_trail, draw_mission));
    }
}

/// Derniers points parcourus.
#[derive(Resource, Default)]
pub struct Trail {
    points: VecDeque<Vec3>,
    clock: f32,
}

const TRAIL_LEN: usize = 360;

impl Trail {
    /// Ajoute un point au sillage.
    pub fn push(&mut self, p: Vec3) {
        self.points.push_back(p);
        while self.points.len() > TRAIL_LEN {
            self.points.pop_front();
        }
    }
}

fn configure_gizmos(mut store: ResMut<GizmoConfigStore>) {
    let (config, _) = store.config_mut::<DefaultGizmoConfigGroup>();
    config.line.width = 1.6;
}

fn record_trail(time: Res<Time>, flight: Res<Flight>, mut trail: ResMut<Trail>) {
    trail.clock += time.delta_secs();
    if trail.clock < 1.0 / 40.0 {
        return;
    }
    trail.clock = 0.0;
    let s = &flight.quad.state;
    if s.on_ground {
        if !trail.points.is_empty() && s.altitude() < 0.05 {
            trail.points.pop_front();
        }
        return;
    }
    let p = s.position();
    trail.points.push_back(Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32));
    while trail.points.len() > TRAIL_LEN {
        trail.points.pop_front();
    }
}

fn draw_trail(trail: Res<Trail>, mut gizmos: Gizmos) {
    let n = trail.points.len();
    if n < 2 {
        return;
    }
    let ember = theme::ember().to_srgba();
    gizmos.linestrip_gradient(trail.points.iter().enumerate().map(|(i, p)| {
        let t = i as f32 / (n - 1) as f32;
        (*p, Color::srgba(ember.red, ember.green, ember.blue, t * t * 0.9))
    }));
}

fn catmull_rom(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3, t: f32) -> Vec3 {
    0.5 * ((2.0 * p1)
        + (-p0 + p2) * t
        + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t * t
        + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t * t * t)
}

fn draw_mission(flight: Res<Flight>, mut gizmos: Gizmos) {
    let mission = &flight.autopilot.mission;
    let wps = mission.waypoints();
    if wps.len() < 2 {
        return;
    }
    let pts: Vec<Vec3> = wps
        .iter()
        .map(|w| Vec3::new(w.position[0] as f32, w.position[1] as f32, w.position[2] as f32))
        .collect();
    let n = pts.len();
    let active = matches!(flight.autopilot.mode(), FlightMode::Mission);
    let current = mission.current_index();

    // Chemin lissé, très discret.
    let path_color = Color::srgba(0.1, 0.1, 0.16, if active { 0.16 } else { 0.10 });
    let mut line: Vec<Vec3> = Vec::with_capacity(n * 12 + 1);
    for i in 0..n {
        let (p0, p1, p2, p3) = if mission.looping {
            (pts[(i + n - 1) % n], pts[i], pts[(i + 1) % n], pts[(i + 2) % n])
        } else {
            (
                pts[i.saturating_sub(1)],
                pts[i],
                pts[(i + 1).min(n - 1)],
                pts[(i + 2).min(n - 1)],
            )
        };
        if !mission.looping && i == n - 1 {
            break;
        }
        for k in 0..12 {
            line.push(catmull_rom(p0, p1, p2, p3, k as f32 / 12.0));
        }
    }
    if mission.looping {
        line.push(line[0]);
    } else {
        line.push(pts[n - 1]);
    }
    gizmos.linestrip(line, path_color);

    // Points de passage : un anneau à plat et un fil jusqu'au sol.
    for (i, p) in pts.iter().enumerate() {
        let is_target = active && i == current;
        let (r, color) = if is_target {
            (0.22, theme::ember())
        } else {
            (0.13, Color::srgba(0.1, 0.1, 0.16, 0.30))
        };
        gizmos.circle(
            Isometry3d::new(*p, Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
            r,
            color,
        );
        gizmos.line(
            *p - Vec3::Y * r.min(0.13) * 0.0,
            Vec3::new(p.x, 0.0, p.z),
            Color::srgba(0.1, 0.1, 0.16, if is_target { 0.22 } else { 0.07 }),
        );
    }
}
