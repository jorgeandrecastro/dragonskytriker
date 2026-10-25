//! Le Dragon X1 : un galet blanc mat, quatre bras fins, des hélices graphite.
//! Tout est fait de primitives ; aucune ressource externe.

use bevy::prelude::*;

use crate::sim::Flight;
use crate::theme;
use dragonskytriker_core::QuadParams;

/// Racine du drone ; sa `Transform` est pilotée par la simulation.
#[derive(Component)]
pub struct DroneRoot;

/// Une hélice qui tourne selon la poussée de son moteur.
#[derive(Component)]
pub struct Propeller {
    /// Indice du moteur (0 avant-droit, 1 avant-gauche, 2 arrière-droit, 3 arrière-gauche).
    pub motor: usize,
    /// Sens de rotation autour de +Y.
    pub direction: f32,
}

/// Modèle 3D et synchronisation avec la simulation.
pub struct DroneModelPlugin;

impl Plugin for DroneModelPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_drone)
            .add_systems(Update, (sync_pose, spin_propellers));
    }
}

fn spawn_drone(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let p = QuadParams::default();
    let d = p.arm_xz() as f32;

    let white = materials.add(StandardMaterial {
        base_color: Color::srgb(0.90, 0.905, 0.92),
        perceptual_roughness: 0.55,
        ..default()
    });
    let graphite = materials.add(StandardMaterial {
        base_color: Color::srgb(0.13, 0.13, 0.15),
        perceptual_roughness: 0.42,
        metallic: 0.5,
        ..default()
    });
    let gloss = materials.add(StandardMaterial {
        base_color: Color::srgb(0.03, 0.03, 0.04),
        perceptual_roughness: 0.12,
        metallic: 0.2,
        ..default()
    });
    let blade = materials.add(StandardMaterial {
        base_color: Color::srgb(0.16, 0.16, 0.18),
        perceptual_roughness: 0.6,
        ..default()
    });
    let disc = materials.add(StandardMaterial {
        base_color: Color::srgba(0.35, 0.36, 0.4, 0.10),
        alpha_mode: AlphaMode::Blend,
        unlit: true,
        ..default()
    });
    let led_front = materials.add(StandardMaterial {
        base_color: theme::ember(),
        emissive: LinearRgba::new(6.0, 1.6, 0.5, 1.0),
        unlit: true,
        ..default()
    });
    let led_rear = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        emissive: LinearRgba::new(3.0, 3.0, 3.2, 1.0),
        unlit: true,
        ..default()
    });

    let pebble = meshes.add(Sphere::new(1.0));
    let arm_mesh = meshes.add(Cuboid::new(0.011, 0.007, p.arm as f32));
    let motor_mesh = meshes.add(Cylinder::new(0.017, 0.024));
    let blade_mesh = meshes.add(Cuboid::new(0.17, 0.0016, 0.013));
    let disc_mesh = meshes.add(Cylinder::new(0.088, 0.0006));
    let led_mesh = meshes.add(Sphere::new(0.0055));
    let lens_mesh = meshes.add(Sphere::new(0.021));

    // (x, z, sens de rotation de l'hélice, est à l'avant)
    let motors = [
        (d, -d, 1.0_f32, true),
        (-d, -d, -1.0, true),
        (d, d, -1.0, false),
        (-d, d, 1.0, false),
    ];

    commands
        .spawn((DroneRoot, Transform::default(), Visibility::default()))
        .with_children(|root| {
            // Corps : une sphère aplatie.
            root.spawn((
                Mesh3d(pebble),
                MeshMaterial3d(white.clone()),
                Transform::from_scale(Vec3::new(0.082, 0.030, 0.082)),
            ));
            // Œil : capteur avant, légèrement plongeant.
            root.spawn((
                Mesh3d(lens_mesh),
                MeshMaterial3d(gloss),
                Transform::from_xyz(0.0, -0.004, -0.066).with_scale(Vec3::new(1.0, 0.9, 1.0)),
            ));

            for (i, (x, z, dir, front)) in motors.into_iter().enumerate() {
                let angle = x.atan2(z);
                // Bras.
                root.spawn((
                    Mesh3d(arm_mesh.clone()),
                    MeshMaterial3d(white.clone()),
                    Transform::from_xyz(x * 0.5, 0.0, z * 0.5)
                        .with_rotation(Quat::from_rotation_y(angle)),
                ));
                // Moteur.
                root.spawn((
                    Mesh3d(motor_mesh.clone()),
                    MeshMaterial3d(graphite.clone()),
                    Transform::from_xyz(x, 0.006, z),
                ));
                // Voyant au bout du bras.
                let r = 1.0 + 0.030 / (x * x + z * z).sqrt();
                root.spawn((
                    Mesh3d(led_mesh.clone()),
                    MeshMaterial3d(if front { led_front.clone() } else { led_rear.clone() }),
                    Transform::from_xyz(x * r, 0.0, z * r),
                ));
                // Hélice : pivot + pale + disque de flou.
                root.spawn((
                    Propeller { motor: i, direction: dir },
                    Transform::from_xyz(x, 0.022, z),
                    Visibility::default(),
                ))
                .with_children(|prop| {
                    prop.spawn((Mesh3d(blade_mesh.clone()), MeshMaterial3d(blade.clone())));
                });
                root.spawn((
                    Mesh3d(disc_mesh.clone()),
                    MeshMaterial3d(disc.clone()),
                    Transform::from_xyz(x, 0.0235, z),
                ));
            }
        });
}

/// Copie la pose simulée vers la `Transform` (quaternion issu de `tiny-rodriguez-nostd`).
fn sync_pose(flight: Res<Flight>, mut q: Query<&mut Transform, With<DroneRoot>>) {
    let pose = &flight.quad.state.pose;
    let qn = pose.rotation.to_quaternion();
    let t = pose.translation;
    for mut tr in &mut q {
        tr.translation = Vec3::new(t[0] as f32, t[1] as f32, t[2] as f32);
        tr.rotation = Quat::from_xyzw(qn[0] as f32, qn[1] as f32, qn[2] as f32, qn[3] as f32);
    }
}

/// Vitesse d'affichage volontairement plafonnée : pas d'effet stroboscopique.
fn spin_propellers(
    time: Res<Time>,
    flight: Res<Flight>,
    mut props: Query<(&Propeller, &mut Transform)>,
) {
    let max = flight.quad.params.max_motor_thrust as f32;
    for (p, mut tr) in &mut props {
        let load = (flight.quad.state.motors[p.motor] as f32 / max).clamp(0.0, 1.0);
        let speed = if load > 0.02 { 10.0 + 46.0 * load.sqrt() } else { 0.0 };
        tr.rotate_local_y(p.direction * speed * time.delta_secs());
    }
}
