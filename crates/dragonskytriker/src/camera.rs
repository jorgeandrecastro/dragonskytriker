//! Caméra cinématographique : trois modes (suivi, orbite, panorama), mouvements amortis.

use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::input::keyboard::Key;
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::pbr::{DistanceFog, FogFalloff};
use bevy::prelude::*;

use crate::input::just;
use crate::sim::Flight;
use crate::theme;

/// Mode de caméra.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum CameraMode {
    /// Derrière le drone, cap lissé.
    #[default]
    Chase,
    /// Orbite autour du drone (glisser pour tourner, molette pour zoomer).
    Orbit,
    /// Plan large fixe qui suit le drone du regard.
    Overview,
}

impl CameraMode {
    /// Mode suivant.
    pub fn next(self) -> Self {
        match self {
            Self::Chase => Self::Orbit,
            Self::Orbit => Self::Overview,
            Self::Overview => Self::Chase,
        }
    }

    /// Nom affiché.
    pub fn label(self) -> &'static str {
        match self {
            Self::Chase => "Chase",
            Self::Orbit => "Orbit",
            Self::Overview => "Overview",
        }
    }
}

/// État de la caméra.
#[derive(Resource)]
pub struct CameraRig {
    /// Mode courant.
    pub mode: CameraMode,
    focus: Vec3,
    chase_yaw: f32,
    orbit_yaw: f32,
    orbit_pitch: f32,
    zoom: f32,
    needs_snap: bool,
}

impl Default for CameraRig {
    fn default() -> Self {
        Self {
            mode: CameraMode::Chase,
            focus: Vec3::new(0.0, 0.3, 0.0),
            chase_yaw: 0.0,
            orbit_yaw: 0.6,
            orbit_pitch: 0.28,
            zoom: 1.0,
            needs_snap: true,
        }
    }
}

/// Marqueur de la caméra principale.
#[derive(Component)]
pub struct MainCamera;

/// Caméra, brouillard et lumière ambiante.
pub struct CameraPlugin;

impl Plugin for CameraPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<CameraRig>()
            .add_systems(Startup, spawn_camera)
            .add_systems(Update, (switch_mode, follow));
    }
}

fn spawn_camera(mut commands: Commands) {
    commands.spawn((
        MainCamera,
        Camera3d::default(),
        Msaa::Sample4,
        // Pas de tonemapping : les couleurs du sol, du brouillard et du fond coïncident exactement.
        Tonemapping::None,
        DistanceFog {
            color: theme::background(),
            falloff: FogFalloff::Linear { start: 16.0, end: 60.0 },
            ..default()
        },
        Transform::from_xyz(0.0, 1.6, 4.0).looking_at(Vec3::new(0.0, 0.5, 0.0), Vec3::Y),
    ));
    commands.insert_resource(GlobalAmbientLight {
        color: Color::srgb(0.93, 0.95, 1.0),
        brightness: 780.0,
        ..default()
    });
}

fn switch_mode(keys: Res<ButtonInput<Key>>, mut rig: ResMut<CameraRig>) {
    if just(&keys, "c") {
        rig.mode = rig.mode.next();
    }
}

fn damp(rate: f32, dt: f32) -> f32 {
    1.0 - (-rate * dt).exp()
}

fn wrap_pi(a: f32) -> f32 {
    let two_pi = std::f32::consts::TAU;
    (a + std::f32::consts::PI).rem_euclid(two_pi) - std::f32::consts::PI
}

fn follow(
    time: Res<Time>,
    flight: Res<Flight>,
    mouse: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    mut rig: ResMut<CameraRig>,
    mut cam: Query<&mut Transform, With<MainCamera>>,
) {
    let Ok(mut tr) = cam.single_mut() else { return };
    let dt = time.delta_secs().min(0.1);
    let s = &flight.quad.state;
    let p = s.position();
    let target_focus = Vec3::new(p[0] as f32, p[1] as f32 + 0.05, p[2] as f32);
    let yaw = s.yaw() as f32;

    rig.zoom = (rig.zoom * (1.0 - scroll.delta.y * 0.08)).clamp(0.4, 3.5);
    let f = damp(10.0, dt);
    rig.focus = rig.focus.lerp(target_focus, f);

    let (desired_pos, look_at) = match rig.mode {
        CameraMode::Chase => {
            let d = wrap_pi(yaw - rig.chase_yaw);
            rig.chase_yaw += d * damp(2.4, dt);
            let fwd = Vec3::new(-rig.chase_yaw.sin(), 0.0, -rig.chase_yaw.cos());
            let pos = rig.focus - fwd * 1.9 * rig.zoom + Vec3::Y * (0.55 * rig.zoom);
            (pos, rig.focus + fwd * 0.6)
        }
        CameraMode::Orbit => {
            if mouse.pressed(MouseButton::Left) {
                rig.orbit_yaw -= motion.delta.x * 0.006;
                rig.orbit_pitch = (rig.orbit_pitch + motion.delta.y * 0.005).clamp(0.03, 1.35);
            } else {
                rig.orbit_yaw += 0.10 * dt;
            }
            let r = 2.5 * rig.zoom;
            let (sy, cy) = rig.orbit_yaw.sin_cos();
            let (sp, cp) = rig.orbit_pitch.sin_cos();
            (rig.focus + Vec3::new(sy * cp, sp, cy * cp) * r, rig.focus)
        }
        CameraMode::Overview => (Vec3::new(13.0, 8.5, 16.0) * rig.zoom, rig.focus),
    };

    let pos_rate = match rig.mode {
        CameraMode::Chase => 5.5,
        _ => 4.0,
    };
    if rig.needs_snap {
        // Premier plan : on se place directement, sans glisser depuis l'origine.
        rig.needs_snap = false;
        rig.focus = target_focus;
        rig.chase_yaw = yaw;
        *tr = Transform::from_translation(desired_pos).looking_at(look_at, Vec3::Y);
        return;
    }
    tr.translation = tr.translation.lerp(desired_pos, damp(pos_rate, dt));
    let desired_rot = Transform::from_translation(tr.translation)
        .looking_at(look_at, Vec3::Y)
        .rotation;
    tr.rotation = tr.rotation.slerp(desired_rot, damp(8.0, dt));
}