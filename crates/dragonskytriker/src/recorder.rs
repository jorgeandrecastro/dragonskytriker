//! Enregistreur de vol : `F` démarre/arrête l'écriture de `flight_log.csv`.

use std::fs::File;
use std::io::{BufWriter, Write};

use bevy::input::keyboard::Key;
use bevy::prelude::*;

use crate::input::just;
use crate::sim::Flight;

/// Écrit la télémétrie (vérité, estimation, batterie, capteurs) à 20 Hz.
pub struct RecorderPlugin;

impl Plugin for RecorderPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Recorder>().add_systems(FixedUpdate, record);
        app.add_systems(Update, toggle);
    }
}

/// Fichier de journal en cours, s'il y en a un.
#[derive(Resource, Default)]
pub struct Recorder {
    file: Option<BufWriter<File>>,
    tick: u32,
}

impl Recorder {
    /// Vrai pendant l'enregistrement.
    pub fn is_recording(&self) -> bool {
        self.file.is_some()
    }
}

fn toggle(keys: Res<ButtonInput<Key>>, mut rec: ResMut<Recorder>) {
    if !just(&keys, "f") {
        return;
    }
    if rec.file.take().is_some() {
        info!("Enregistrement terminé : flight_log.csv");
        return;
    }
    match File::create("flight_log.csv") {
        Ok(f) => {
            let mut w = BufWriter::new(f);
            let _ = writeln!(
                w,
                "t,x,y,z,est_x,est_y,est_z,speed,tilt_deg,soc,voltage,power_w,nearest_lidar_m"
            );
            rec.file = Some(w);
            info!("Enregistrement démarré : flight_log.csv");
        }
        Err(e) => warn!("Impossible de créer flight_log.csv : {e}"),
    }
}

fn record(flight: Res<Flight>, mut rec: ResMut<Recorder>) {
    rec.tick += 1;
    if rec.tick % 12 != 0 {
        return;
    }
    let q = &flight.quad.state;
    let (p, e) = (q.position(), flight.estimator.position());
    let nearest = flight.sensors.latest.lidar.nearest().unwrap_or(f32::NAN);
    if let Some(w) = rec.file.as_mut() {
        let _ = writeln!(
            w,
            "{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.2},{:.4},{:.2},{:.1},{:.2}",
            flight.time,
            p[0],
            p[1],
            p[2],
            e[0],
            e[1],
            e[2],
            q.speed(),
            q.tilt().to_degrees(),
            q.battery.soc(),
            q.battery.voltage(),
            q.power,
            nearest
        );
    }
}