//! Capteurs simulés : IMU, baromètre, magnétomètre, GNSS, télémètre, lidar.
//!
//! Chaque capteur a son bruit, ses biais lents (marche aléatoire), sa fréquence ;
//! le GNSS a une erreur corrélée et une latence. Tout est déterministe (graine).

use crate::dynamics::QuadState;
use crate::environment::Environment;
use crate::rng::Rng;
use crate::world::Collider;
use libm::{cos, exp, sin, sqrt};
use tiny_rodriguez_nostd::linalg::{add, scale};
use tiny_rodriguez_nostd::{Pose3, Vec3};

/// Nombre de rayons horizontaux du lidar (2° de résolution).
pub const LIDAR_H: usize = 180;
/// Nombre de canaux verticaux du lidar.
pub const LIDAR_V: usize = 4;

// ---------------------------------------------------------------------------
// IMU
// ---------------------------------------------------------------------------

/// Paramètres de l'IMU (valeurs d'une puce MEMS courante, type MPU-6000).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuConfig {
    /// Densité de bruit du gyroscope (rad/s/√Hz).
    pub gyro_noise_density: f64,
    /// Marche aléatoire du biais gyroscopique (rad/s/√s).
    pub gyro_bias_walk: f64,
    /// Écart-type du biais gyroscopique initial (rad/s).
    pub gyro_bias_initial: f64,
    /// Densité de bruit de l'accéléromètre (m/s²/√Hz).
    pub accel_noise_density: f64,
    /// Marche aléatoire du biais d'accéléromètre (m/s²/√s).
    pub accel_bias_walk: f64,
    /// Écart-type du biais d'accéléromètre initial (m/s²).
    pub accel_bias_initial: f64,
}

impl Default for ImuConfig {
    fn default() -> Self {
        Self {
            gyro_noise_density: 8.7e-5,
            gyro_bias_walk: 1.0e-5,
            gyro_bias_initial: 0.004,
            accel_noise_density: 3.9e-3,
            accel_bias_walk: 1.0e-4,
            accel_bias_initial: 0.04,
        }
    }
}

/// Mesure de l'IMU, repère corps.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ImuSample {
    /// Force spécifique (m/s²).
    pub accel: Vec3,
    /// Vitesse angulaire (rad/s).
    pub gyro: Vec3,
}

/// Centrale inertielle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Imu {
    /// Paramètres.
    pub config: ImuConfig,
    gyro_bias: Vec3,
    accel_bias: Vec3,
    rng: Rng,
}

impl Imu {
    /// Crée une IMU avec des biais initiaux tirés au hasard.
    pub fn new(config: ImuConfig, seed: u64) -> Self {
        let mut rng = Rng::new(seed);
        let gyro_bias = rng.gaussian3(config.gyro_bias_initial);
        let accel_bias = rng.gaussian3(config.accel_bias_initial);
        Self { config, gyro_bias, accel_bias, rng }
    }

    /// Biais réels (pour comparer à l'estimation).
    pub fn true_gyro_bias(&self) -> Vec3 {
        self.gyro_bias
    }

    /// Mesure sur un pas `dt`.
    pub fn sample(&mut self, s: &QuadState, dt: f64) -> ImuSample {
        let c = &self.config;
        let sq = sqrt(dt);
        for k in 0..3 {
            self.gyro_bias[k] += c.gyro_bias_walk * sq * self.rng.gaussian();
            self.accel_bias[k] += c.accel_bias_walk * sq * self.rng.gaussian();
        }
        let gn = c.gyro_noise_density / sq;
        let an = c.accel_noise_density / sq;
        let mut out = ImuSample::default();
        for k in 0..3 {
            out.gyro[k] = s.omega[k] + self.gyro_bias[k] + gn * self.rng.gaussian();
            out.accel[k] = s.specific_force[k] + self.accel_bias[k] + an * self.rng.gaussian();
        }
        out
    }
}

// ---------------------------------------------------------------------------
// Baromètre
// ---------------------------------------------------------------------------

/// Mesure du baromètre.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct BaroSample {
    /// Pression (Pa).
    pub pressure: f64,
    /// Température (K).
    pub temperature: f64,
    /// Altitude relative à l'origine du monde, déduite de la pression (m).
    pub altitude: f64,
}

/// Baromètre : bruit blanc + dérive lente.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Barometer {
    /// Bruit (Pa).
    pub noise: f64,
    /// Marche aléatoire de la dérive (Pa/√s).
    pub drift_walk: f64,
    drift: f64,
    rng: Rng,
}

impl Barometer {
    /// Baromètre type MS5611.
    pub fn new(seed: u64) -> Self {
        Self { noise: 1.5, drift_walk: 0.05, drift: 0.0, rng: Rng::new(seed) }
    }

    /// Mesure sur un intervalle `dt` depuis la mesure précédente.
    pub fn sample(&mut self, s: &QuadState, env: &Environment, dt: f64) -> BaroSample {
        self.drift += self.drift_walk * sqrt(dt) * self.rng.gaussian();
        let h = env.origin_altitude + s.altitude();
        let p = env.atmosphere.pressure_at(h) + self.drift + self.noise * self.rng.gaussian();
        BaroSample {
            pressure: p,
            temperature: env.atmosphere.temperature_at(h) + 0.3 * self.rng.gaussian(),
            altitude: env.atmosphere.altitude_from_pressure(p) - env.origin_altitude,
        }
    }
}

// ---------------------------------------------------------------------------
// Magnétomètre
// ---------------------------------------------------------------------------

/// Magnétomètre : champ terrestre dans le repère corps + biais fer dur + bruit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Magnetometer {
    /// Bruit (µT).
    pub noise: f64,
    hard_iron: Vec3,
    rng: Rng,
}

impl Magnetometer {
    /// Magnétomètre avec un biais fer dur aléatoire d'environ 1 µT.
    pub fn new(seed: u64) -> Self {
        let mut rng = Rng::new(seed);
        let hard_iron = rng.gaussian3(1.0);
        Self { noise: 0.3, hard_iron, rng }
    }

    /// Champ mesuré (µT), repère corps.
    pub fn sample(&mut self, s: &QuadState, env: &Environment) -> Vec3 {
        let b = s.pose.rotation.inverse().rotate(&env.magnetic_field);
        let n = self.rng.gaussian3(self.noise);
        [b[0] + self.hard_iron[0] + n[0], b[1] + self.hard_iron[1] + n[1], b[2] + self.hard_iron[2] + n[2]]
    }
}

// ---------------------------------------------------------------------------
// GNSS
// ---------------------------------------------------------------------------

/// Position GNSS.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct GnssFix {
    /// Position dans le repère monde (m), bruitée.
    pub position: Vec3,
    /// Vitesse dans le repère monde (m/s), bruitée.
    pub velocity: Vec3,
    /// Latitude (°).
    pub latitude: f64,
    /// Longitude (°).
    pub longitude: f64,
    /// Altitude au-dessus du niveau de la mer (m).
    pub altitude_msl: f64,
    /// Nombre de satellites.
    pub satellites: u8,
    /// Instant de la mesure (s de simulation).
    pub timestamp: f64,
}

/// Récepteur GNSS : erreur corrélée dans le temps (Gauss-Markov) et latence.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gnss {
    /// Écart-type horizontal (m).
    pub horizontal_sigma: f64,
    /// Écart-type vertical (m).
    pub vertical_sigma: f64,
    /// Écart-type de la vitesse (m/s).
    pub velocity_sigma: f64,
    /// Temps de corrélation de l'erreur de position (s).
    pub correlation_time: f64,
    /// Latence de sortie (s).
    pub latency: f64,
    err: Vec3,
    queue: [GnssFix; 8],
    queue_len: usize,
    rng: Rng,
}

impl Gnss {
    /// Récepteur 10 Hz ordinaire : ±0,6 m horizontal, ±1 m vertical, 0,12 s de latence.
    pub fn new(seed: u64) -> Self {
        Self {
            horizontal_sigma: 0.6,
            vertical_sigma: 1.0,
            velocity_sigma: 0.06,
            correlation_time: 30.0,
            latency: 0.12,
            err: [0.0; 3],
            queue: [GnssFix::default(); 8],
            queue_len: 0,
            rng: Rng::new(seed),
        }
    }

    /// Prend une mesure (à la cadence du récepteur). Elle sortira après `latency`.
    pub fn measure(&mut self, s: &QuadState, env: &Environment, interval: f64, time: f64) {
        let a = exp(-interval / self.correlation_time);
        let k = sqrt(1.0 - a * a);
        let sig = [self.horizontal_sigma, self.vertical_sigma, self.horizontal_sigma];
        for i in 0..3 {
            self.err[i] = a * self.err[i] + k * sig[i] * self.rng.gaussian();
        }
        let position = add(&s.position(), &self.err);
        let vn = self.rng.gaussian3(self.velocity_sigma);
        let velocity = add(&s.velocity, &vn);
        let (latitude, longitude) = env.to_geodetic(&position);
        let fix = GnssFix {
            position,
            velocity,
            latitude,
            longitude,
            altitude_msl: env.origin_altitude + position[1],
            satellites: 14,
            timestamp: time,
        };
        if self.queue_len == self.queue.len() {
            self.queue.copy_within(1.., 0);
            self.queue_len -= 1;
        }
        self.queue[self.queue_len] = fix;
        self.queue_len += 1;
    }

    /// Retourne la mesure arrivée à maturité (plus ancienne que la latence), s'il y en a une.
    pub fn poll(&mut self, time: f64) -> Option<GnssFix> {
        if self.queue_len > 0 && time >= self.queue[0].timestamp + self.latency {
            let fix = self.queue[0];
            self.queue.copy_within(1.., 0);
            self.queue_len -= 1;
            Some(fix)
        } else {
            None
        }
    }
}

// ---------------------------------------------------------------------------
// Télémètre et lidar
// ---------------------------------------------------------------------------

/// Télémètre orienté vers le bas (−Y du corps).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rangefinder {
    /// Portée maximale (m).
    pub max_range: f64,
    /// Bruit relatif (fraction de la distance).
    pub relative_noise: f64,
    rng: Rng,
}

impl Rangefinder {
    /// Télémètre 12 m, 1 % de bruit.
    pub fn new(seed: u64) -> Self {
        Self { max_range: 12.0, relative_noise: 0.01, rng: Rng::new(seed) }
    }

    /// Distance au premier obstacle sous le drone, ou `None` hors de portée.
    pub fn measure<C: Collider>(&mut self, s: &QuadState, world: &C) -> Option<f64> {
        let dir = s.pose.rotation.rotate(&[0.0, -1.0, 0.0]);
        let hit = world.raycast(&s.position(), &dir, self.max_range)?;
        Some(hit.distance * (1.0 + self.relative_noise * self.rng.gaussian()) + 0.01 * self.rng.gaussian())
    }
}

/// Configuration du lidar.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LidarConfig {
    /// Portée maximale (m).
    pub max_range: f64,
    /// Bruit sur la distance (m).
    pub noise: f64,
    /// Élévation du premier canal (rad).
    pub elevation_min: f64,
    /// Élévation du dernier canal (rad).
    pub elevation_max: f64,
}

impl Default for LidarConfig {
    fn default() -> Self {
        Self { max_range: 12.0, noise: 0.02, elevation_min: -0.30, elevation_max: 0.30 }
    }
}

/// Un balayage lidar : `LIDAR_V` canaux de `LIDAR_H` rayons sur 360°.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LidarScan {
    /// Distances (m) ; `f32::INFINITY` si aucun retour.
    pub ranges: [[f32; LIDAR_H]; LIDAR_V],
    /// Pose du capteur au moment du balayage.
    pub pose: Pose3,
    /// Instant du balayage (s).
    pub timestamp: f64,
}

impl Default for LidarScan {
    fn default() -> Self {
        Self { ranges: [[f32::INFINITY; LIDAR_H]; LIDAR_V], pose: Pose3::identity(), timestamp: 0.0 }
    }
}

impl LidarScan {
    /// Distance la plus courte du balayage (m).
    pub fn nearest(&self) -> Option<f32> {
        let mut m = f32::INFINITY;
        for row in &self.ranges {
            for &r in row {
                if r < m {
                    m = r;
                }
            }
        }
        if m.is_finite() {
            Some(m)
        } else {
            None
        }
    }
}

/// Lidar 360° multi-canaux.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Lidar {
    /// Paramètres.
    pub config: LidarConfig,
    rng: Rng,
}

impl Lidar {
    /// Crée un lidar.
    pub fn new(config: LidarConfig, seed: u64) -> Self {
        Self { config, rng: Rng::new(seed) }
    }

    /// Direction du rayon `(h, v)` dans le repère corps. L'azimut 0 est vers l'avant (−Z).
    pub fn ray_direction(&self, h: usize, v: usize) -> Vec3 {
        let a = -core::f64::consts::PI + core::f64::consts::TAU * h as f64 / LIDAR_H as f64;
        let e = if LIDAR_V == 1 {
            0.0
        } else {
            let t = v as f64 / (LIDAR_V - 1) as f64;
            self.config.elevation_min + t * (self.config.elevation_max - self.config.elevation_min)
        };
        [sin(a) * cos(e), sin(e), -cos(a) * cos(e)]
    }

    /// Effectue un balayage complet.
    pub fn scan<C: Collider>(&mut self, s: &QuadState, world: &C, time: f64) -> LidarScan {
        let origin = add(&s.position(), &s.pose.rotation.rotate(&[0.0, 0.04, 0.0]));
        let mut out = LidarScan { pose: s.pose, timestamp: time, ..LidarScan::default() };
        for v in 0..LIDAR_V {
            for h in 0..LIDAR_H {
                let dir = s.pose.rotation.rotate(&self.ray_direction(h, v));
                if let Some(hit) = world.raycast(&origin, &dir, self.config.max_range) {
                    let n = self.config.noise * self.rng.gaussian();
                    out.ranges[v][h] = (hit.distance + n).max(0.0) as f32;
                }
            }
        }
        out
    }
}

// ---------------------------------------------------------------------------
// Ensemble de capteurs
// ---------------------------------------------------------------------------

/// Dernières mesures de chaque capteur.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SensorReadings {
    /// IMU.
    pub imu: ImuSample,
    /// Baromètre.
    pub baro: BaroSample,
    /// Magnétomètre (µT, repère corps).
    pub mag: Vec3,
    /// Dernière position GNSS reçue.
    pub gnss: Option<GnssFix>,
    /// Télémètre vers le bas (m).
    pub range_down: Option<f64>,
    /// Dernier balayage lidar.
    pub lidar: LidarScan,
}

/// Quels capteurs ont produit une nouvelle mesure pendant ce pas.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SensorEvents {
    /// Nouvelle mesure IMU.
    pub imu: bool,
    /// Nouvelle mesure baromètre.
    pub baro: bool,
    /// Nouvelle mesure magnétomètre.
    pub mag: bool,
    /// Nouvelle mesure GNSS.
    pub gnss: bool,
    /// Nouvelle mesure télémètre.
    pub range: bool,
    /// Nouveau balayage lidar.
    pub lidar: bool,
}

/// Fréquences d'échantillonnage (Hz).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensorRates {
    /// Baromètre.
    pub baro: f64,
    /// Magnétomètre.
    pub mag: f64,
    /// GNSS.
    pub gnss: f64,
    /// Télémètre.
    pub range: f64,
    /// Lidar.
    pub lidar: f64,
}

impl Default for SensorRates {
    fn default() -> Self {
        Self { baro: 50.0, mag: 50.0, gnss: 10.0, range: 30.0, lidar: 10.0 }
    }
}

/// Tous les capteurs du drone, avec leurs cadences.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sensors {
    /// IMU (échantillonnée à chaque pas de physique).
    pub imu: Imu,
    /// Baromètre.
    pub baro: Barometer,
    /// Magnétomètre.
    pub mag: Magnetometer,
    /// GNSS.
    pub gnss: Gnss,
    /// Télémètre.
    pub rangefinder: Rangefinder,
    /// Lidar.
    pub lidar: Lidar,
    /// Cadences.
    pub rates: SensorRates,
    /// Dernières mesures.
    pub latest: SensorReadings,
    timers: [f64; 5],
    time: f64,
}

impl Sensors {
    /// Ensemble de capteurs déterministe pour une graine donnée.
    pub fn new(seed: u64) -> Self {
        Self {
            imu: Imu::new(ImuConfig::default(), seed ^ 0x11),
            baro: Barometer::new(seed ^ 0x22),
            mag: Magnetometer::new(seed ^ 0x33),
            gnss: Gnss::new(seed ^ 0x44),
            rangefinder: Rangefinder::new(seed ^ 0x55),
            lidar: Lidar::new(LidarConfig::default(), seed ^ 0x66),
            rates: SensorRates::default(),
            latest: SensorReadings::default(),
            timers: [0.0; 5],
            time: 0.0,
        }
    }

    /// Avance les capteurs de `dt` et met à jour [`Sensors::latest`].
    pub fn update<C: Collider>(
        &mut self,
        s: &QuadState,
        env: &Environment,
        world: &C,
        dt: f64,
    ) -> SensorEvents {
        self.time += dt;
        let mut ev = SensorEvents { imu: true, ..SensorEvents::default() };
        self.latest.imu = self.imu.sample(s, dt);

        let periods = [
            1.0 / self.rates.baro,
            1.0 / self.rates.mag,
            1.0 / self.rates.gnss,
            1.0 / self.rates.range,
            1.0 / self.rates.lidar,
        ];
        for i in 0..5 {
            self.timers[i] += dt;
        }
        if self.timers[0] >= periods[0] {
            self.latest.baro = self.baro.sample(s, env, self.timers[0]);
            self.timers[0] = 0.0;
            ev.baro = true;
        }
        if self.timers[1] >= periods[1] {
            self.latest.mag = self.mag.sample(s, env);
            self.timers[1] = 0.0;
            ev.mag = true;
        }
        if self.timers[2] >= periods[2] {
            self.gnss.measure(s, env, self.timers[2], self.time);
            self.timers[2] = 0.0;
        }
        if let Some(fix) = self.gnss.poll(self.time) {
            self.latest.gnss = Some(fix);
            ev.gnss = true;
        }
        if self.timers[3] >= periods[3] {
            self.latest.range_down = self.rangefinder.measure(s, world);
            self.timers[3] = 0.0;
            ev.range = true;
        }
        if self.timers[4] >= periods[4] {
            self.latest.lidar = self.lidar.scan(s, world, self.time);
            self.timers[4] = 0.0;
            ev.lidar = true;
        }
        ev
    }

    /// Remet les capteurs à zéro (biais retirés au sort avec la même graine).
    pub fn reset(&mut self, seed: u64) {
        *self = Self::new(seed);
    }

    /// Décale un point selon une direction (utilitaire de visualisation).
    pub fn point_along(origin: &Vec3, dir: &Vec3, distance: f64) -> Vec3 {
        add(origin, &scale(dir, distance))
    }
}
