use crate::autopilot::Autopilot;
use crate::control::Gains;
use crate::dynamics::{Air, QuadState, Quadrotor};
use crate::environment::Environment;
use crate::estimator::{Estimator, EstimatorConfig};
use crate::params::QuadParams;
use crate::sensors::{SensorEvents, Sensors};
use crate::world::World;

/// Capacité du monde (obstacles).
pub const WORLD_CAPACITY: usize = 32;
/// Capacité d'une mission (points de passage).
pub const MISSION_CAPACITY: usize = 16;

/// La simulation complète : drone, autopilote, environnement, monde, capteurs, estimateur.
///
/// Un seul appel, [`Simulation::step`], fait tout avancer. Aucune dépendance à un moteur
/// de rendu : on peut l'exécuter sans fenêtre, plus vite que le temps réel.
#[derive(Debug, Clone, Copy)]
pub struct Simulation {
    /// Drone simulé (vérité terrain).
    pub quad: Quadrotor,
    /// Autopilote.
    pub autopilot: Autopilot<MISSION_CAPACITY>,
    /// Environnement (vent, atmosphère, champ magnétique, pesanteur).
    pub env: Environment,
    /// Obstacles.
    pub world: World<WORLD_CAPACITY>,
    /// Capteurs.
    pub sensors: Sensors,
    /// Estimateur d'état.
    pub estimator: Estimator,
    /// Si vrai, l'autopilote vole sur l'état **estimé** par les capteurs ; sinon sur la vérité.
    pub use_estimator: bool,
    /// Temps de simulation (s).
    pub time: f64,
    /// Capteurs ayant produit une mesure au dernier pas.
    pub events: SensorEvents,
    seed: u64,
}

impl Simulation {
    /// Simulation neuve, déterministe pour une graine donnée.
    pub fn new(params: QuadParams, gains: Gains, seed: u64) -> Self {
        let env = Environment::default();
        let mut sim = Self {
            quad: Quadrotor::new(params),
            autopilot: Autopilot::new(params, gains),
            env,
            world: World::new(),
            sensors: Sensors::new(seed),
            estimator: Estimator::new(EstimatorConfig::default(), &env.magnetic_field),
            use_estimator: false,
            time: 0.0,
            events: SensorEvents::default(),
            seed,
        };
        sim.estimator.reset_to(&sim.quad.state);
        sim
    }

    /// État donné à l'autopilote (estimé ou vrai).
    pub fn control_state(&self) -> QuadState {
        if self.use_estimator {
            self.estimator.control_state(&self.quad.state)
        } else {
            self.quad.state
        }
    }

    /// Replace le drone au sol et réinitialise capteurs, estimateur et batterie.
    pub fn reset(&mut self) {
        self.quad.reset([0.0; 3], 0.0);
        self.sensors.reset(self.seed);
        self.estimator = Estimator::new(EstimatorConfig::default(), &self.env.magnetic_field);
        self.estimator.reset_to(&self.quad.state);
        let (params, gains) = (self.quad.params, self.autopilot.controller.gains);
        let mission = self.autopilot.mission;
        self.autopilot = Autopilot::new(params, gains);
        self.autopilot.mission = mission;
        self.autopilot.mission.restart();
        self.time = 0.0;
    }

    /// Avance toute la simulation de `dt` secondes.
    pub fn step(&mut self, dt: f64) {
        // 1. Autopilote → moteurs.
        let cs = self.control_state();
        let motors = self.autopilot.update(&cs, dt);

        // 2. Environnement et physique.
        self.env.step(dt);
        let air = Air {
            wind: self.env.wind.velocity(),
            density: self.env.atmosphere.density_at(self.env.origin_altitude + self.quad.state.altitude()),
        };
        self.quad.step_with(&motors, dt, &air, &self.world, self.env.gravity);
        self.time += dt;

        // 3. Capteurs.
        self.events = self.sensors.update(&self.quad.state, &self.env, &self.world, dt);

        // 4. Estimation : prédiction à chaque mesure d'IMU, corrections à la cadence de chaque capteur.
        if self.quad.state.on_ground && self.autopilot.mode() == crate::autopilot::FlightMode::Disarmed {
            // Au sol, avant le vol : alignement sur la position connue.
            let truth = self.quad.state;
            self.estimator.reset_to(&truth);
        } else {
            self.estimator.predict(&self.sensors.latest.imu, dt);
            if self.events.mag {
                let m = self.sensors.latest.mag;
                self.estimator.correct_mag(&m, 1.0 / self.sensors.rates.mag);
            }
            if self.events.baro {
                let b = self.sensors.latest.baro;
                self.estimator.correct_baro(&b, 1.0 / self.sensors.rates.baro);
            }
            if self.events.gnss {
                if let Some(fix) = self.sensors.latest.gnss {
                    self.estimator.correct_gnss(&fix, 1.0 / self.sensors.rates.gnss);
                }
            }
        }
    }
}

impl Default for Simulation {
    fn default() -> Self {
        Self::new(QuadParams::default(), Gains::default(), 1)
    }
}
