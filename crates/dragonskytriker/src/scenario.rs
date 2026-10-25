//! Fichiers de monde (JSON) : obstacles, vent et mission, à la manière d'un fichier SDF.
//!
//! ```json
//! {
//!   "name": "Slalom",
//!   "wind": { "mean": [2.0, 0.0, 0.0], "gust_sigma": 0.8 },
//!   "obstacles": [ { "Cylinder": { "base": [3.0, 0.0, 0.0], "radius": 0.4, "height": 4.0 } } ],
//!   "mission": { "looping": true, "waypoints": [ { "position": [0.0, 2.0, 0.0] } ] }
//! }
//! ```
//! Lancer avec `DST_WORLD=mon_monde.json cargo run --release`.

use dragonskytriker_core::{Shape, Simulation, Waypoint, Wind};
use serde::{Deserialize, Serialize};

/// Vent d'un monde.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct WindSpec {
    /// Vent moyen (m/s), repère monde.
    #[serde(default)]
    pub mean: [f64; 3],
    /// Écart-type des rafales (m/s).
    #[serde(default)]
    pub gust_sigma: f64,
}

/// Point de passage d'un fichier de monde.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WaypointSpec {
    /// Position (m).
    pub position: [f64; 3],
    /// Cap (rad). S'il est absent, le drone regarde dans le sens de la marche.
    #[serde(default)]
    pub yaw: Option<f64>,
    /// Maintien (s).
    #[serde(default)]
    pub hold: f64,
}

/// Mission d'un fichier de monde.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct MissionSpec {
    /// Recommencer à la fin.
    #[serde(default)]
    pub looping: bool,
    /// Points de passage.
    #[serde(default)]
    pub waypoints: Vec<WaypointSpec>,
}

/// Un monde complet.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Scenario {
    /// Nom.
    #[serde(default)]
    pub name: String,
    /// Vent.
    #[serde(default)]
    pub wind: WindSpec,
    /// Obstacles.
    #[serde(default)]
    pub obstacles: Vec<Shape>,
    /// Mission.
    #[serde(default)]
    pub mission: MissionSpec,
}

impl Scenario {
    /// Charge un monde depuis un fichier JSON.
    pub fn load(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(&std::fs::read_to_string(path)?)?)
    }

    /// Écrit le monde dans un fichier JSON.
    pub fn save(&self, path: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(path, serde_json::to_string_pretty(self)?)?;
        Ok(())
    }

    /// Applique le monde à une simulation.
    pub fn apply(&self, sim: &mut Simulation) {
        sim.world.clear();
        for s in &self.obstacles {
            if sim.world.push(*s).is_err() {
                eprintln!("DragonSkyTriker : trop d'obstacles (maximum {}).", dragonskytriker_core::WORLD_CAPACITY);
                break;
            }
        }
        sim.env.wind = Wind::new(self.wind.mean, self.wind.gust_sigma, 7);
        let m = &mut sim.autopilot.mission;
        m.clear();
        m.looping = self.mission.looping;
        let pts = &self.mission.waypoints;
        for (i, w) in pts.iter().enumerate() {
            let yaw = w.yaw.unwrap_or_else(|| {
                let next = &pts[(i + 1) % pts.len()].position;
                let (dx, dz) = (next[0] - w.position[0], next[2] - w.position[2]);
                (-dx).atan2(-dz)
            });
            let _ = m.push(Waypoint { position: w.position, yaw, hold: w.hold });
        }
    }

    /// La scène de démonstration : un huit couché entouré de piliers et de volumes.
    pub fn default_scene() -> Self {
        let n = 12;
        let waypoints = (0..n)
            .map(|i| {
                let t = std::f64::consts::TAU * i as f64 / n as f64;
                let (x, z) = (4.5 * t.sin(), -3.0 * (2.0 * t).sin());
                let (dx, dz) = (4.5 * t.cos(), -6.0 * (2.0 * t).cos());
                WaypointSpec { position: [x, 2.1 + 0.5 * t.sin(), z], yaw: Some((-dx).atan2(-dz)), hold: 0.0 }
            })
            .collect();
        let pillar = |x: f64, z: f64| Shape::Cylinder { base: [x, 0.0, z], radius: 0.4, height: 4.0 };
        Self {
            name: "Studio".into(),
            wind: WindSpec { mean: [0.0; 3], gust_sigma: 0.0 },
            obstacles: vec![
                pillar(7.0, 2.0),
                pillar(-7.0, -3.0),
                pillar(0.0, -6.5),
                pillar(6.0, -5.0),
                pillar(-5.5, 5.0),
                pillar(3.0, 6.0),
                Shape::Box { center: [-6.0, 0.5, 2.0], half: [0.8, 0.5, 0.8], yaw: 0.4 },
                Shape::Box { center: [5.5, 0.75, 5.0], half: [0.6, 0.75, 1.2], yaw: -0.5 },
                Shape::Sphere { center: [-2.5, 0.6, -7.0], radius: 0.6 },
            ],
            mission: MissionSpec { looping: true, waypoints },
        }
    }
}
