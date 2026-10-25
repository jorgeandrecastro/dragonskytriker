//! Simulation : la ressource [`Flight`] enveloppe la [`Simulation`] du cœur et avance à 240 Hz.

use bevy::prelude::*;
use dragonskytriker_core::{Gains, QuadParams, Simulation};

use crate::scenario::Scenario;

/// Fréquence de la physique (Hz).
pub const PHYSICS_HZ: f64 = 240.0;

/// La simulation complète : drone, autopilote, environnement, monde, capteurs, estimateur.
///
/// `Flight` se déréférence en [`Simulation`] : `flight.quad`, `flight.autopilot`,
/// `flight.world`, `flight.sensors`, `flight.estimator`...
///
/// ```ignore
/// fn my_system(mut flight: ResMut<Flight>) {
///     let state = flight.control_state();
///     flight.autopilot.command(Command::Takeoff { altitude: 3.0 }, &state);
/// }
/// ```
#[derive(Resource, Deref, DerefMut)]
pub struct Flight(pub Simulation);

impl Default for Flight {
    fn default() -> Self {
        let mut sim = Simulation::new(QuadParams::default(), Gains::default(), 1);
        Scenario::default_scene().apply(&mut sim);
        Self(sim)
    }
}

/// Avance la physique, les capteurs et l'autopilote à pas fixe.
pub struct SimPlugin;

impl Plugin for SimPlugin {
    fn build(&self, app: &mut App) {
        let mut flight = Flight::default();
        // DST_WORLD=chemin.json charge un monde personnalisé.
        if let Ok(path) = std::env::var("DST_WORLD") {
            match Scenario::load(&path) {
                Ok(s) => s.apply(&mut flight.0),
                Err(e) => eprintln!("DragonSkyTriker : monde « {path} » illisible ({e}), scène par défaut utilisée."),
            }
        }
        app.insert_resource(Time::<Fixed>::from_hz(PHYSICS_HZ))
            .insert_resource(flight)
            .add_systems(FixedUpdate, step);
    }
}

fn step(mut flight: ResMut<Flight>, time: Res<Time>) {
    flight.step(time.delta_secs_f64());
}
