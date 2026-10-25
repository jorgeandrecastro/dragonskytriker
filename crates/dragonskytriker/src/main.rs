use bevy::prelude::*;
use bevy::render::view::screenshot::{save_to_disk, Screenshot};
use bevy::window::PresentMode;
use dragonskytriker::camera::{CameraMode, CameraRig};
use dragonskytriker::core::Command;
use dragonskytriker::path::Trail;
use dragonskytriker::{DragonSkyPlugins, Flight};

fn main() {
    let mut app = App::new();
    app.add_plugins((
        DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "DragonSkyTriker".into(),
                resolution: (1440, 900).into(),
                present_mode: PresentMode::AutoVsync,
                ..default()
            }),
            ..default()
        }),
        DragonSkyPlugins,
    ));

    // Mode capture (développement) : DST_SCREENSHOT=chemin.png [DST_WARP=secondes] [DST_CAMERA=chase|orbit|overview]
    if let Ok(path) = std::env::var("DST_SCREENSHOT") {
        // Rendu logiciel lent : chaque image avance d'exactement 1/60 s, comme à 60 ips.
        app.insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
            std::time::Duration::from_secs_f64(1.0 / 60.0),
        ))
        .insert_resource(ShotJob { path, frames: 0 })
            .add_systems(Startup, warp_flight)
            .add_systems(Update, take_shot);
    }
    app.run();
}

#[derive(Resource)]
struct ShotJob {
    path: String,
    frames: u32,
}

/// Avance la simulation sans rendu pour obtenir un drone déjà en vol.
fn warp_flight(mut flight: ResMut<Flight>, mut trail: ResMut<Trail>, mut rig: ResMut<CameraRig>) {
    let seconds: f64 = std::env::var("DST_WARP").ok().and_then(|v| v.parse().ok()).unwrap_or(14.0);
    rig.mode = match std::env::var("DST_CAMERA").as_deref() {
        Ok("orbit") => CameraMode::Orbit,
        Ok("overview") => CameraMode::Overview,
        _ => CameraMode::Chase,
    };
    if let Ok(w) = std::env::var("DST_WIND") {
        let v: f64 = w.parse().unwrap_or(0.0);
        flight.env.wind = dragonskytriker::core::Wind::new([v, 0.0, -0.4 * v], 0.3 * v, 5);
    }
    flight.use_estimator = std::env::var("DST_ESTIMATE").is_ok();
    let state = flight.control_state();
    flight.autopilot.command(Command::StartMission, &state);
    let dt = 1.0 / 240.0;
    for i in 0..(seconds * 240.0) as usize {
        flight.step(dt);
        if i % 6 == 0 && !flight.quad.state.on_ground {
            let p = flight.quad.state.position();
            trail.push(Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32));
        }
    }
    println!("warp: mode={:?} pos={:?}", flight.autopilot.mode(), flight.quad.state.position());
}

fn take_shot(mut commands: Commands, mut job: ResMut<ShotJob>, mut exit: MessageWriter<AppExit>) {
    job.frames += 1;
    if job.frames == 150 {
        commands.spawn(Screenshot::primary_window()).observe(save_to_disk(job.path.clone()));
    }
    if job.frames == 200 {
        exit.write(AppExit::Success);
    }
}
