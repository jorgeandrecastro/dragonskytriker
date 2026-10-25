//! Visualisation des capteurs : retours lidar, vent, estimation de position.

use bevy::input::keyboard::Key;
use bevy::prelude::*;

use crate::input::just;
use crate::sim::Flight;
use crate::theme;
use dragonskytriker_core::{LIDAR_H, LIDAR_V};

/// Ce qui est affiché. `V` bascule le tout.
#[derive(Resource)]
pub struct SensorView {
    /// Retours lidar.
    pub lidar: bool,
    /// Flèche du vent.
    pub wind: bool,
    /// Position estimée (fantôme).
    pub estimate: bool,
}

impl Default for SensorView {
    fn default() -> Self {
        Self { lidar: true, wind: true, estimate: true }
    }
}

/// Lidar, vent, estimation.
pub struct SensorsVizPlugin;

impl Plugin for SensorsVizPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SensorView>()
            .add_systems(Update, (toggle, draw_lidar, draw_wind, draw_estimate));
    }
}

fn toggle(keys: Res<ButtonInput<Key>>, mut view: ResMut<SensorView>) {
    if just(&keys, "v") {
        let on = !(view.lidar || view.wind || view.estimate);
        view.lidar = on;
        view.wind = on;
        view.estimate = on;
    }
}

fn v3(v: [f64; 3]) -> Vec3 {
    Vec3::new(v[0] as f32, v[1] as f32, v[2] as f32)
}

fn draw_lidar(flight: Res<Flight>, view: Res<SensorView>, mut gizmos: Gizmos) {
    if !view.lidar {
        return;
    }
    let scan = &flight.sensors.latest.lidar;
    let rot = &scan.pose.rotation;
    let origin = v3(scan.pose.translation) + v3(rot.rotate(&[0.0, 0.04, 0.0]));
    let max = flight.sensors.lidar.config.max_range as f32;
    for v in 0..LIDAR_V {
        for h in 0..LIDAR_H {
            let r = scan.ranges[v][h];
            if !r.is_finite() {
                continue;
            }
            let dir = v3(rot.rotate(&flight.sensors.lidar.ray_direction(h, v)));
            let p = origin + dir * r;
            if p.y < 0.03 {
                continue; // le sol n'est pas un obstacle
            }
            let t = (r / max).clamp(0.0, 1.0);
            let color = Color::srgba(1.0 - 0.55 * t, 0.42 + 0.08 * t, 0.24 + 0.38 * t, 0.9 - 0.45 * t);
            gizmos.line(p, p - dir * 0.05, color);
        }
    }
}

fn draw_wind(flight: Res<Flight>, view: Res<SensorView>, mut gizmos: Gizmos) {
    if !view.wind {
        return;
    }
    let w = v3(flight.env.wind.velocity());
    if w.length() < 0.25 {
        return;
    }
    let start = v3(flight.quad.state.position()) + Vec3::Y * 0.3;
    let vec = w.clamp_length_max(8.0) * 0.28;
    let tip = start + vec;
    let color = Color::srgba(0.1, 0.1, 0.16, 0.38);
    gizmos.line(start, tip, color);
    let dir = vec.normalize_or_zero();
    let side = dir.cross(Vec3::Y).normalize_or_zero();
    let head = (vec.length() * 0.3).clamp(0.05, 0.14);
    gizmos.line(tip, tip - dir * head + side * head * 0.55, color);
    gizmos.line(tip, tip - dir * head - side * head * 0.55, color);
}

fn draw_estimate(flight: Res<Flight>, view: Res<SensorView>, mut gizmos: Gizmos) {
    if !view.estimate {
        return;
    }
    let truth = v3(flight.quad.state.position());
    let est = v3(flight.estimator.position());
    if flight.quad.state.on_ground {
        return;
    }
    let color = theme::ember().with_alpha(0.75);
    gizmos.circle(
        Isometry3d::new(est, Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
        0.16,
        color,
    );
    gizmos.line(truth, est, theme::ember().with_alpha(0.25));
}