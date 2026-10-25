//! # dragonskytriker-core
//!
//! Cœur `#![no_std]` du SDK **DragonSkyTriker** : tout ce qui n'a pas besoin
//! d'un moteur de rendu. Le même code peut tourner dans une simulation Bevy
//! et, plus tard, dans le firmware d'un vrai drone.
//!
//! - [`Quadrotor`] : dynamique d'un quadricoptère (corps rigide, moteurs du
//!   premier ordre, traînée), intégrée sur SE(3) avec `tiny-rodriguez-nostd`.
//! - [`Controller`] : contrôle géométrique en cascade (position → attitude sur
//!   SO(3) → moteurs).
//! - [`Mixer`] : allocation poussée/couples → poussées des 4 moteurs.
//! - [`Mission`] et [`Autopilot`] : points de passage et modes de vol.
//!
//! ## Repère
//!
//! Même convention que Bevy : repère **main droite, Y vers le haut**.
//! Le drone regarde vers **−Z**, sa poussée s'exerce selon son axe **+Y**.
//! La gravité vaut `(0, −g, 0)`. Le lacet est une rotation autour de +Y.
//! Unités SI (m, s, kg, rad).

#![no_std]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

mod autopilot;
mod battery;
mod environment;
mod estimator;
mod rng;
mod sensors;
mod simulation;
mod world;
mod control;
mod dynamics;
mod mission;
mod mixer;
mod params;

pub use autopilot::{Autopilot, Command, FlightMode};
pub use control::{Controller, Gains, Setpoint};
pub use battery::Battery;
pub use dynamics::{yaw_of, Air, QuadState, Quadrotor};
pub use environment::{Atmosphere, Environment, Wind};
pub use estimator::{Estimator, EstimatorConfig};
pub use rng::Rng;
pub use sensors::{
    BaroSample, Barometer, Gnss, GnssFix, Imu, ImuConfig, ImuSample, Lidar, LidarConfig,
    LidarScan, Magnetometer, Rangefinder, SensorEvents, SensorRates, SensorReadings, Sensors,
    LIDAR_H, LIDAR_V,
};
pub use simulation::{Simulation, MISSION_CAPACITY, WORLD_CAPACITY};
pub use world::{Collider, Contact, EmptyWorld, RayHit, Shape, World};
pub use mission::{Mission, Waypoint};
pub use mixer::Mixer;
pub use params::QuadParams;

pub use tiny_rodriguez_nostd as geometry;
pub use tiny_rodriguez_nostd::{Pose3, Rotation3, Vec3};

/// Accélération de la pesanteur (m/s²).
pub const GRAVITY: f64 = 9.81;
