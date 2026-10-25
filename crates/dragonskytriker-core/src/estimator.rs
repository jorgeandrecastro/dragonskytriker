//! Estimation d'état à partir des capteurs.
//!
//! - **Attitude** : filtre de Mahony sur SO(3) (gyroscope + gravité + champ magnétique),
//!   avec estimation du biais gyroscopique. L'intégration passe par `Rotation3::exp`.
//! - **Position / vitesse** : filtre complémentaire (accéléromètre en prédiction,
//!   GNSS et baromètre en correction, latence GNSS compensée).

use crate::dynamics::QuadState;
use crate::sensors::{BaroSample, GnssFix, ImuSample};
use libm::{atan2, exp, fabs, sqrt};
use tiny_rodriguez_nostd::linalg::{add, cross, dot, norm, scale, sub};
use tiny_rodriguez_nostd::{Pose3, Rotation3, Vec3};

/// Taille de l'historique d'accélération (≈ 0,5 s à 240 Hz).
const HIST: usize = 128;

/// Gains de l'estimateur.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EstimatorConfig {
    /// Gain proportionnel de l'attitude (gravité), 1/s.
    pub attitude_kp: f64,
    /// Gain intégral (biais gyroscopique), 1/s².
    pub attitude_ki: f64,
    /// Gain du cap magnétique, 1/s.
    pub heading_gain: f64,
    /// Gain de recalage de la position par GNSS, 1/s.
    pub gnss_position_gain: f64,
    /// Gain de recalage de la vitesse par GNSS, 1/s.
    pub gnss_velocity_gain: f64,
    /// Gain de recalage de l'altitude par baromètre, 1/s.
    pub baro_gain: f64,
    /// Latence du GNSS à compenser (s).
    pub gnss_latency: f64,
    /// Pesanteur supposée (m/s²).
    pub gravity: f64,
}

impl Default for EstimatorConfig {
    fn default() -> Self {
        Self {
            attitude_kp: 1.5,
            attitude_ki: 0.03,
            heading_gain: 0.8,
            gnss_position_gain: 3.0,
            gnss_velocity_gain: 2.0,
            baro_gain: 2.5,
            gnss_latency: 0.12,
            gravity: 9.81,
        }
    }
}

/// Estimateur d'état du drone.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Estimator {
    /// Gains.
    pub config: EstimatorConfig,
    attitude: Rotation3,
    gyro_bias: Vec3,
    position: Vec3,
    velocity: Vec3,
    omega: Vec3,
    mag_reference: Vec3,
    baro_offset: f64,
    baro_initialized: bool,
    /// Écart lent entre l'accélération vraie (vue par le GNSS) et celle déduite de l'IMU.
    accel_error: Vec3,
    prev_gnss: Option<(Vec3, f64)>,
    clock: f64,
    hist_t: [f64; HIST],
    hist_dv: [Vec3; HIST],
    hist_i: usize,
}

impl Estimator {
    /// Crée un estimateur. `magnetic_field` est le champ terrestre du monde (pour le cap).
    pub fn new(config: EstimatorConfig, magnetic_field: &Vec3) -> Self {
        let h = [magnetic_field[0], 0.0, magnetic_field[2]];
        let n = norm(&h);
        Self {
            config,
            attitude: Rotation3::identity(),
            gyro_bias: [0.0; 3],
            position: [0.0; 3],
            velocity: [0.0; 3],
            omega: [0.0; 3],
            mag_reference: if n > 1e-9 { scale(&h, 1.0 / n) } else { [0.0, 0.0, -1.0] },
            baro_offset: 0.0,
            baro_initialized: false,
            accel_error: [0.0; 3],
            prev_gnss: None,
            clock: 0.0,
            hist_t: [f64::NEG_INFINITY; HIST],
            hist_dv: [[0.0; 3]; HIST],
            hist_i: 0,
        }
    }

    /// Initialise l'estimateur sur l'état vrai (drone aligné au sol avant le vol).
    pub fn reset_to(&mut self, truth: &QuadState) {
        self.attitude = truth.pose.rotation;
        self.position = truth.position();
        self.velocity = truth.velocity;
        self.omega = truth.omega;
        self.gyro_bias = [0.0; 3];
        self.baro_initialized = false;
        self.accel_error = [0.0; 3];
        self.prev_gnss = None;
        self.clock = 0.0;
        self.hist_t = [f64::NEG_INFINITY; HIST];
        self.hist_i = 0;
    }

    /// Attitude estimée.
    pub fn attitude(&self) -> Rotation3 {
        self.attitude
    }

    /// Position estimée (m).
    pub fn position(&self) -> Vec3 {
        self.position
    }

    /// Vitesse estimée, repère monde (m/s).
    pub fn velocity(&self) -> Vec3 {
        self.velocity
    }

    /// Biais gyroscopique estimé (rad/s).
    pub fn gyro_bias(&self) -> Vec3 {
        self.gyro_bias
    }

    /// Prédiction : intègre un échantillon d'IMU sur `dt`.
    pub fn predict(&mut self, imu: &ImuSample, dt: f64) {
        let c = &self.config;
        let g = c.gravity;

        // Accélération linéaire estimée : IMU + attitude courante, corrigée de l'écart lent vu par le GNSS.
        let a_est = add(&self.attitude.rotate(&imu.accel), &[0.0, -g, 0.0]);
        let a_lin = add(&a_est, &self.accel_error);

        // Attitude : correction par la direction de la gravité. Un quadricoptère en manœuvre
        // mesure surtout sa poussée ; en retirant l'accélération linéaire on retrouve la gravité.
        let lin_body = self.attitude.inverse().rotate(&a_lin);
        let grav = [imu.accel[0] - lin_body[0], imu.accel[1] - lin_body[1], imu.accel[2] - lin_body[2]];
        let an = norm(&grav);
        let mut e = [0.0; 3];
        if an > 1e-6 {
            let trust = (1.0 - fabs(an - g) / (0.3 * g)).clamp(0.0, 1.0);
            let a_hat = scale(&grav, 1.0 / an);
            let up_body = self.attitude.inverse().rotate(&[0.0, 1.0, 0.0]);
            e = scale(&cross(&a_hat, &up_body), trust);
        }
        // Historique des variations de vitesse prédites (pour comparer au GNSS sur la même fenêtre).
        self.clock += dt;
        self.hist_t[self.hist_i] = self.clock;
        self.hist_dv[self.hist_i] = scale(&a_est, dt);
        self.hist_i = (self.hist_i + 1) % HIST;
        for k in 0..3 {
            self.gyro_bias[k] = (self.gyro_bias[k] - c.attitude_ki * e[k] * dt).clamp(-0.2, 0.2);
        }
        let w = [
            imu.gyro[0] - self.gyro_bias[0] + c.attitude_kp * e[0],
            imu.gyro[1] - self.gyro_bias[1] + c.attitude_kp * e[1],
            imu.gyro[2] - self.gyro_bias[2] + c.attitude_kp * e[2],
        ];
        self.attitude = self.attitude.integrate(&w, dt);
        if self.attitude.orthogonality_error() > 1e-9 {
            self.attitude = self.attitude.renormalized();
        }
        self.omega = [
            imu.gyro[0] - self.gyro_bias[0],
            imu.gyro[1] - self.gyro_bias[1],
            imu.gyro[2] - self.gyro_bias[2],
        ];

        // Position / vitesse : navigation à l'estime.
        let a_world = add(&self.attitude.rotate(&imu.accel), &[0.0, -g, 0.0]);
        for k in 0..3 {
            self.position[k] += self.velocity[k] * dt + 0.5 * a_world[k] * dt * dt;
            self.velocity[k] += a_world[k] * dt;
        }
    }

    /// Correction du cap par le magnétomètre (µT, repère corps).
    pub fn correct_mag(&mut self, mag_body: &Vec3, dt: f64) {
        let m_w = self.attitude.rotate(mag_body);
        let m_h = [m_w[0], 0.0, m_w[2]];
        let n = norm(&m_h);
        if n < 1e-6 {
            return;
        }
        let m_h = scale(&m_h, 1.0 / n);
        let r = self.mag_reference;
        let err = atan2(cross(&m_h, &r)[1], dot(&m_h, &r));
        let k = (1.0 - exp(-self.config.heading_gain * dt)).clamp(0.0, 1.0);
        let corr = Rotation3::exp(&[0.0, err * k, 0.0]);
        self.attitude = corr.compose(&self.attitude);
    }

    /// Correction par une mesure GNSS (position et vitesse).
    pub fn correct_gnss(&mut self, fix: &GnssFix, dt: f64) {
        let c = &self.config;
        // Compense la latence : la mesure décrit le drone il y a `gnss_latency` secondes.
        let pos = add(&fix.position, &scale(&fix.velocity, c.gnss_latency));
        // Écart d'accélération : variation de vitesse GNSS − variation prédite par l'IMU,
        // sur la fenêtre [t_prev, t_fix] (la latence s'annule), filtré passe-bas.
        if let Some((v_prev, t_prev)) = self.prev_gnss {
            let span = fix.timestamp - t_prev;
            if span > 1e-3 {
                let mut dv_est = [0.0; 3];
                for i in 0..HIST {
                    let t = self.hist_t[i];
                    if t > t_prev && t <= fix.timestamp {
                        dv_est = add(&dv_est, &self.hist_dv[i]);
                    }
                }
                let dv_gnss = sub(&fix.velocity, &v_prev);
                for k in 0..3 {
                    let d = (dv_gnss[k] - dv_est[k]) / span;
                    self.accel_error[k] += 0.2 * (d - self.accel_error[k]);
                }
            }
        }
        self.prev_gnss = Some((fix.velocity, fix.timestamp));
        let kp = 1.0 - exp(-c.gnss_position_gain * dt);
        let kv = 1.0 - exp(-c.gnss_velocity_gain * dt);
        for k in 0..3 {
            // Verticale : le baromètre est meilleur, le GNSS n'intervient que faiblement.
            let w = if k == 1 { 0.2 } else { 1.0 };
            self.position[k] += w * kp * (pos[k] - self.position[k]);
            self.velocity[k] += w * kv * (fix.velocity[k] - self.velocity[k]);
        }
    }

    /// Correction de l'altitude par le baromètre.
    pub fn correct_baro(&mut self, baro: &BaroSample, dt: f64) {
        if !self.baro_initialized {
            // Le zéro du baromètre est pris à l'initialisation (comme un vrai drone au sol).
            self.baro_offset = baro.altitude - self.position[1];
            self.baro_initialized = true;
        }
        let alt = baro.altitude - self.baro_offset;
        let k = 1.0 - exp(-self.config.baro_gain * dt);
        let err = alt - self.position[1];
        self.position[1] += k * err;
        self.velocity[1] += 0.5 * k * err / dt.max(1e-3) * dt;
    }

    /// État pour le contrôleur : pose, vitesse et rotation estimées ; le reste (moteurs,
    /// batterie, contact sol) vient de `truth` car le firmware le connaît directement.
    pub fn control_state(&self, truth: &QuadState) -> QuadState {
        QuadState {
            pose: Pose3::new(self.attitude, self.position),
            velocity: self.velocity,
            omega: self.omega,
            ..*truth
        }
    }

    /// Erreur d'attitude par rapport à la vérité (rad).
    pub fn attitude_error(&self, truth: &QuadState) -> f64 {
        truth.pose.rotation.geodesic_distance(&self.attitude)
    }

    /// Erreur de position par rapport à la vérité (m).
    pub fn position_error(&self, truth: &QuadState) -> f64 {
        let d = sub(&self.position, &truth.position());
        sqrt(dot(&d, &d))
    }
}
