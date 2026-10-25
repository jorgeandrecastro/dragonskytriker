//! Contrôles clavier : ordres de haut niveau et téléopération.

use bevy::input::keyboard::Key;
use bevy::prelude::*;

use crate::sim::Flight;
use dragonskytriker_core::{Command, FlightMode, Setpoint, Wind};

/// Clavier → autopilote.
pub struct ControlsPlugin;

impl Plugin for ControlsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Teleop>().add_systems(Update, handle_keys);
    }
}

/// Point de consigne intégré à partir des touches de déplacement.
#[derive(Resource, Default)]
pub struct Teleop {
    active: bool,
    target: Vec3,
    yaw: f32,
}

const TAKEOFF_ALTITUDE: f64 = 2.0;
const SPEED_H: f32 = 3.0;
const SPEED_V: f32 = 1.4;
const YAW_RATE: f32 = 1.6;

/// Touche caractère, insensible à la casse et à la disposition du clavier.
fn is_char(k: &Key, c: &str) -> bool {
    matches!(k, Key::Character(s) if s.eq_ignore_ascii_case(c))
}

fn just(keys: &ButtonInput<Key>, c: &str) -> bool {
    keys.get_just_pressed().any(|k| is_char(k, c))
}

fn held(keys: &ButtonInput<Key>, cs: &[&str]) -> bool {
    keys.get_pressed().any(|k| cs.iter().any(|c| is_char(k, c)))
}

fn handle_keys(
    keys: Res<ButtonInput<Key>>,
    time: Res<Time>,
    mut flight: ResMut<Flight>,
    mut tele: ResMut<Teleop>,
) {
    // Les ordres sont donnés dans le repère de ce que l'autopilote croit : vérité ou estimation.
    let st = flight.control_state();
    let sim = &mut flight.0;
    let autopilot = &mut sim.autopilot;

    if just(&keys, "t") {
        autopilot.command(Command::Takeoff { altitude: TAKEOFF_ALTITUDE }, &st);
    }
    if just(&keys, "l") {
        autopilot.command(Command::Land, &st);
    }
    if just(&keys, "h") {
        autopilot.command(Command::Hover, &st);
    }
    if just(&keys, "m") {
        autopilot.command(Command::StartMission, &st);
    }
    if just(&keys, "r") {
        sim.reset();
        tele.active = false;
        return;
    }
    if just(&keys, "e") {
        sim.use_estimator = !sim.use_estimator;
    }
    if just(&keys, "g") {
        // Air calme → brise → rafales → tempête.
        let presets = [
            ([0.0, 0.0, 0.0], 0.0),
            ([2.5, 0.0, -1.0], 0.0),
            ([3.0, 0.0, -1.5], 1.2),
            ([5.5, 0.0, -2.0], 2.2),
        ];
        let cur = sim.env.wind.mean[0];
        let next = presets
            .iter()
            .position(|p| (p.0[0] - cur).abs() < 1e-9)
            .map(|i| (i + 1) % presets.len())
            .unwrap_or(0);
        sim.env.wind = Wind::new(presets[next].0, presets[next].1, 21);
    }

    // Téléopération : ZQSD (AZERTY) ou WASD (QWERTY), Espace/↑ monter, ↓ descendre, ←/→ cap.
    let b = |x: bool| x as i32 as f32;
    let fwd = b(held(&keys, &["w", "z"])) - b(held(&keys, &["s"]));
    let right = b(held(&keys, &["d"])) - b(held(&keys, &["a", "q"]));
    let up = b(keys.pressed(Key::Space) || keys.pressed(Key::ArrowUp))
        - b(keys.pressed(Key::ArrowDown));
    let yaw_in = b(keys.pressed(Key::ArrowLeft)) - b(keys.pressed(Key::ArrowRight));
    let any = fwd != 0.0 || right != 0.0 || up != 0.0 || yaw_in != 0.0;

    if any {
        if autopilot.mode() == FlightMode::Disarmed {
            autopilot.command(Command::Takeoff { altitude: 1.5 }, &st);
            return;
        }
        if matches!(autopilot.mode(), FlightMode::Takeoff | FlightMode::Land) {
            return;
        }
        let p = st.position();
        let pos = Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32);
        if !tele.active {
            tele.active = true;
            tele.target = pos;
            tele.yaw = st.yaw() as f32;
        }
        let dt = time.delta_secs();
        tele.yaw += yaw_in * YAW_RATE * dt;
        let (sy, cy) = tele.yaw.sin_cos();
        let forward = Vec3::new(-sy, 0.0, -cy);
        let rightv = Vec3::new(cy, 0.0, -sy);
        let vel = (forward * fwd + rightv * right).normalize_or_zero() * SPEED_H
            + Vec3::Y * up * SPEED_V;
        tele.target += vel * dt;
        // Le point de consigne reste à portée du drone (pas d'emballement).
        let off = (tele.target - pos).clamp_length_max(1.4);
        tele.target = pos + off;
        tele.target.y = tele.target.y.max(0.3);
        autopilot.command(
            Command::Manual(Setpoint {
                position: [tele.target.x as f64, tele.target.y as f64, tele.target.z as f64],
                velocity: [vel.x as f64, vel.y as f64, vel.z as f64],
                yaw: tele.yaw as f64,
            }),
            &st,
        );
    } else if tele.active {
        tele.active = false;
        autopilot.command(Command::Hover, &st);
    }
}