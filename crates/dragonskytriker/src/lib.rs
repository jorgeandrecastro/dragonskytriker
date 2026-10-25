//! # DragonSkyTriker
//!
//! SDK de drones pour Bevy, au design épuré. La physique et le contrôle vivent
//! dans [`dragonskytriker_core`] (`no_std`, basé sur `tiny-rodriguez-nostd`) ;
//! cette crate apporte le rendu, la caméra, l'interface et les contrôles.
//!
//! ```no_run
//! use bevy::prelude::*;
//! use dragonskytriker::DragonSkyPlugins;
//!
//! App::new().add_plugins((DefaultPlugins, DragonSkyPlugins)).run();
//! ```

pub mod camera;
pub mod controls;
pub mod drone_model;
pub mod hud;
pub mod input;
pub mod path;
pub mod recorder;
pub mod scenario;
pub mod sensors_viz;
pub mod sim;
pub mod studio;
pub mod theme;

pub use dragonskytriker_core as core;
pub use sim::Flight;

use bevy::app::PluginGroupBuilder;
use bevy::prelude::*;

/// Tout DragonSkyTriker en une ligne : simulation, studio, drone, caméra, HUD,
/// contrôles clavier et tracé de mission.
pub struct DragonSkyPlugins;

impl PluginGroup for DragonSkyPlugins {
    fn build(self) -> PluginGroupBuilder {
        PluginGroupBuilder::start::<Self>()
            .add(sim::SimPlugin)
            .add(studio::StudioPlugin)
            .add(drone_model::DroneModelPlugin)
            .add(camera::CameraPlugin)
            .add(path::PathPlugin)
            .add(sensors_viz::SensorsVizPlugin)
            .add(hud::HudPlugin)
            .add(controls::ControlsPlugin)
            .add(recorder::RecorderPlugin)
    }
}