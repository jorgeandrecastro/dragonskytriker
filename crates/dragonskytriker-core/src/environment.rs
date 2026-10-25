//! Environnement : atmosphère standard, vent avec rafales, champ magnétique, origine GPS.

use crate::rng::Rng;
use crate::GRAVITY;
use tiny_rodriguez_nostd::Vec3;

/// Vent moyen + rafales (processus d'Ornstein-Uhlenbeck).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Wind {
    /// Vent moyen, repère monde (m/s).
    pub mean: Vec3,
    /// Écart-type des rafales par axe (m/s).
    pub gust_sigma: f64,
    /// Constante de temps des rafales (s).
    pub gust_tau: f64,
    gust: Vec3,
    rng: Rng,
}

impl Wind {
    /// Air calme.
    pub const fn calm() -> Self {
        Self { mean: [0.0; 3], gust_sigma: 0.0, gust_tau: 2.0, gust: [0.0; 3], rng: Rng::new(7) }
    }

    /// Vent moyen `mean` avec des rafales d'écart-type `gust_sigma`.
    pub const fn new(mean: Vec3, gust_sigma: f64, seed: u64) -> Self {
        Self { mean, gust_sigma, gust_tau: 2.0, gust: [0.0; 3], rng: Rng::new(seed) }
    }

    /// Avance les rafales de `dt` (discrétisation exacte de l'OU).
    pub fn step(&mut self, dt: f64) {
        if self.gust_sigma <= 0.0 {
            return;
        }
        let a = libm::exp(-dt / self.gust_tau);
        let s = self.gust_sigma * libm::sqrt(1.0 - a * a);
        for k in 0..3 {
            // Rafales verticales plus faibles que les horizontales.
            let scale = if k == 1 { 0.4 } else { 1.0 };
            self.gust[k] = a * self.gust[k] + s * scale * self.rng.gaussian();
        }
    }

    /// Vitesse de l'air (monde), rafales comprises.
    pub fn velocity(&self) -> Vec3 {
        [self.mean[0] + self.gust[0], self.mean[1] + self.gust[1], self.mean[2] + self.gust[2]]
    }
}

/// Atmosphère standard internationale (troposphère).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Atmosphere {
    /// Pression au niveau de la mer (Pa).
    pub sea_level_pressure: f64,
    /// Température au niveau de la mer (K).
    pub sea_level_temperature: f64,
}

impl Default for Atmosphere {
    fn default() -> Self {
        Self { sea_level_pressure: 101_325.0, sea_level_temperature: 288.15 }
    }
}

impl Atmosphere {
    /// Température (K) à l'altitude `h` (m).
    pub fn temperature_at(&self, h: f64) -> f64 {
        self.sea_level_temperature - 0.0065 * h
    }

    /// Pression (Pa) à l'altitude `h` (m).
    pub fn pressure_at(&self, h: f64) -> f64 {
        let t = self.temperature_at(h) / self.sea_level_temperature;
        self.sea_level_pressure * libm::pow(t, 5.255_88)
    }

    /// Altitude (m) correspondant à une pression (Pa).
    pub fn altitude_from_pressure(&self, p: f64) -> f64 {
        let r = libm::pow(p / self.sea_level_pressure, 1.0 / 5.255_88);
        self.sea_level_temperature * (1.0 - r) / 0.0065
    }

    /// Masse volumique de l'air (kg/m³) à l'altitude `h`.
    pub fn density_at(&self, h: f64) -> f64 {
        self.pressure_at(h) / (287.058 * self.temperature_at(h))
    }
}

/// L'environnement complet de la simulation.
///
/// Repère : +X est l'est, −Z est le nord, +Y est le haut.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Environment {
    /// Vent.
    pub wind: Wind,
    /// Atmosphère.
    pub atmosphere: Atmosphere,
    /// Champ magnétique terrestre, repère monde (µT).
    pub magnetic_field: Vec3,
    /// Latitude de l'origine du monde (°).
    pub origin_latitude: f64,
    /// Longitude de l'origine du monde (°).
    pub origin_longitude: f64,
    /// Altitude de l'origine au-dessus du niveau de la mer (m).
    pub origin_altitude: f64,
    /// Pesanteur (m/s²).
    pub gravity: f64,
}

impl Default for Environment {
    /// Paris : 48,8566° N, 2,3522° E, champ ≈ 48 µT incliné de 65°.
    fn default() -> Self {
        Self {
            wind: Wind::calm(),
            atmosphere: Atmosphere::default(),
            magnetic_field: [0.8, -43.5, -20.5],
            origin_latitude: 48.8566,
            origin_longitude: 2.3522,
            origin_altitude: 35.0,
            gravity: GRAVITY,
        }
    }
}

impl Environment {
    /// Avance l'environnement.
    pub fn step(&mut self, dt: f64) {
        self.wind.step(dt);
    }

    /// Latitude/longitude (°) d'un point du monde.
    pub fn to_geodetic(&self, p: &Vec3) -> (f64, f64) {
        let north = -p[2];
        let east = p[0];
        let lat = self.origin_latitude + north / 111_320.0;
        let lon = self.origin_longitude
            + east / (111_320.0 * libm::cos(self.origin_latitude.to_radians()));
        (lat, lon)
    }
}
