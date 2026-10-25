//! Le « studio » : un sol blanc infini, des anneaux de repère, une ombre douce.

use bevy::prelude::*;

use crate::sim::Flight;
use crate::theme;

/// Fond, lumière, sol, anneaux et ombre de contact.
pub struct StudioPlugin;

impl Plugin for StudioPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(ClearColor(theme::background()))
            .add_systems(Startup, spawn_studio)
            .add_systems(PostStartup, spawn_obstacles)
            .add_systems(Update, update_contact_shadow);
    }
}

/// Un disque de l'ombre de contact (plusieurs disques superposés = dégradé doux).
#[derive(Component)]
struct ShadowLayer {
    base_alpha: f32,
}

#[derive(Component)]
struct ContactShadow;

fn flat_unlit(color: Color, blend: bool) -> StandardMaterial {
    StandardMaterial {
        base_color: color,
        unlit: true,
        alpha_mode: if blend { AlphaMode::Blend } else { AlphaMode::Opaque },
        ..default()
    }
}

fn spawn_studio(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Lumière : une clé douce venant de l'avant-gauche, un peu de remplissage.
    commands.spawn((
        DirectionalLight {
            color: Color::srgb(1.0, 0.985, 0.96),
            illuminance: 4_200.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(-4.0, 9.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Sol infini (le brouillard de la caméra le fond dans l'arrière-plan).
    let flat = Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);
    commands.spawn((
        Mesh3d(meshes.add(Circle::new(120.0))),
        MeshMaterial3d(materials.add(flat_unlit(theme::ground(), false))),
        Transform::from_rotation(flat),
    ));

    // Anneaux de repère tous les 2 m : une échelle discrète, jamais d'encombrement.
    let ring_mat = materials.add(flat_unlit(theme::ring(), false));
    for i in 1..=6 {
        let r = 2.0 * i as f32;
        commands.spawn((
            Mesh3d(meshes.add(Annulus::new(r - 0.012, r + 0.012))),
            MeshMaterial3d(ring_mat.clone()),
            Transform::from_xyz(0.0, 0.001, 0.0).with_rotation(flat),
        ));
    }

    // Ombre de contact : 7 disques translucides concentriques.
    commands
        .spawn((ContactShadow, Transform::from_xyz(0.0, 0.002, 0.0), Visibility::default()))
        .with_children(|p| {
            for i in 0..7 {
                let k = 1.0 - i as f32 / 7.0;
                let base_alpha = 0.045;
                let mat = materials.add(flat_unlit(Color::srgba(0.1, 0.1, 0.16, base_alpha), true));
                p.spawn((
                    ShadowLayer { base_alpha },
                    Mesh3d(meshes.add(Circle::new(0.05 + 0.30 * k))),
                    MeshMaterial3d(mat),
                    Transform::from_xyz(0.0, 0.0005 * i as f32, 0.0).with_rotation(flat),
                ));
            }
        });
}

/// L'ombre suit le drone au sol, s'élargit et s'estompe avec l'altitude.
fn update_contact_shadow(
    flight: Res<Flight>,
    mut shadow: Query<&mut Transform, With<ContactShadow>>,
    layers: Query<(&ShadowLayer, &MeshMaterial3d<StandardMaterial>)>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let pos = flight.quad.state.position();
    let alt = pos[1] as f32;
    if let Ok(mut t) = shadow.single_mut() {
        t.translation = Vec3::new(pos[0] as f32, 0.002, pos[2] as f32);
        t.scale = Vec3::splat(1.0 + 0.45 * alt);
    }
    let fade = 1.0 / (1.0 + 0.8 * alt);
    for (layer, handle) in &layers {
        if let Some(mut m) = materials.get_mut(&handle.0) {
            m.base_color = Color::srgba(0.1, 0.1, 0.16, layer.base_alpha * fade);
        }
    }
}

/// Les obstacles du monde, en volumes mats et ombres de contact douces.
fn spawn_obstacles(
    mut commands: Commands,
    flight: Res<Flight>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let body = materials.add(StandardMaterial {
        base_color: Color::srgb(0.90, 0.905, 0.93),
        perceptual_roughness: 0.8,
        ..default()
    });
    let flat = Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);
    let v3 = |v: [f64; 3]| Vec3::new(v[0] as f32, v[1] as f32, v[2] as f32);
    let blob = |commands: &mut Commands,
                meshes: &mut Assets<Mesh>,
                materials: &mut Assets<StandardMaterial>,
                x: f32,
                z: f32,
                r: f32| {
        for i in 0..5 {
            let k = 1.0 + 0.9 * (1.0 - i as f32 / 5.0);
            commands.spawn((
                Mesh3d(meshes.add(Circle::new(r * k))),
                MeshMaterial3d(materials.add(flat_unlit(Color::srgba(0.1, 0.1, 0.16, 0.05), true))),
                Transform::from_xyz(x, 0.0015 + 0.0004 * i as f32, z).with_rotation(flat),
            ));
        }
    };
    for shape in flight.world.shapes() {
        match *shape {
            dragonskytriker_core::Shape::Sphere { center, radius } => {
                let c = v3(center);
                commands.spawn((
                    Mesh3d(meshes.add(Sphere::new(radius as f32))),
                    MeshMaterial3d(body.clone()),
                    Transform::from_translation(c),
                ));
                blob(&mut commands, &mut meshes, &mut materials, c.x, c.z, radius as f32 * 0.9);
            }
            dragonskytriker_core::Shape::Box { center, half, yaw } => {
                let (c, h) = (v3(center), v3(half));
                commands.spawn((
                    Mesh3d(meshes.add(Cuboid::new(h.x * 2.0, h.y * 2.0, h.z * 2.0))),
                    MeshMaterial3d(body.clone()),
                    Transform::from_translation(c).with_rotation(Quat::from_rotation_y(yaw as f32)),
                ));
                blob(&mut commands, &mut meshes, &mut materials, c.x, c.z, h.x.max(h.z) * 0.9);
            }
            dragonskytriker_core::Shape::Cylinder { base, radius, height } => {
                let b = v3(base);
                commands.spawn((
                    Mesh3d(meshes.add(Cylinder::new(radius as f32, height as f32))),
                    MeshMaterial3d(body.clone()),
                    Transform::from_xyz(b.x, b.y + height as f32 * 0.5, b.z),
                ));
                blob(&mut commands, &mut meshes, &mut materials, b.x, b.z, radius as f32);
            }
        }
    }
}
