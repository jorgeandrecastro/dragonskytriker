//! Régénère les mondes d'exemple : `cargo run -p dragonskytriker --example export_worlds`.

use dragonskytriker::scenario::{MissionSpec, Scenario, WaypointSpec, WindSpec};
use dragonskytriker_core::Shape;

fn wp(x: f64, y: f64, z: f64) -> WaypointSpec {
    WaypointSpec { position: [x, y, z], yaw: None, hold: 0.0 }
}

fn main() {
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/assets/worlds");
    std::fs::create_dir_all(dir).unwrap();

    Scenario::default_scene().save(&format!("{dir}/studio.json")).unwrap();

    // Slalom : trois piliers, un vent de face, passage alternativement à gauche et à droite.
    let pillar = |x: f64| Shape::Cylinder { base: [x, 0.0, 0.0], radius: 0.4, height: 4.0 };
    let slalom = Scenario {
        name: "Slalom".into(),
        wind: WindSpec { mean: [1.5, 0.0, 0.5], gust_sigma: 0.5 },
        obstacles: vec![pillar(6.0), pillar(12.0), pillar(18.0)],
        mission: MissionSpec {
            looping: false,
            waypoints: vec![
                wp(0.0, 1.8, 0.0),
                wp(4.0, 1.8, 2.2),
                wp(6.0, 1.8, 2.2),
                wp(9.0, 1.8, 0.0),
                wp(12.0, 1.8, -2.2),
                wp(15.0, 1.8, 0.0),
                wp(18.0, 1.8, 2.2),
                wp(22.0, 1.8, 0.0),
            ],
        },
    };
    slalom.save(&format!("{dir}/slalom.json")).unwrap();
    println!("Mondes écrits dans {dir}");
}
