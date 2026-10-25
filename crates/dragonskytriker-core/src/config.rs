//! Configuration externe d'un essai : drone, batterie, gains, vent, air, graine.
//!
//! Tout est optionnel : un champ absent garde sa valeur par défaut. La validation
//! refuse les valeurs absurdes avant de construire la simulation.

use core::fmt;

use crate::battery::Battery;
use crate::control::Gains;
use crate::params::QuadParams;
use crate::simulation::Simulation;
use crate::Wind;

/// Erreur de configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ConfigError {
    /// Valeur NaN ou infinie.
    NotFinite(&'static str),
    /// Valeur qui doit être strictement positive.
    NotPositive(&'static str),
    /// Valeur qui doit être positive ou nulle.
    Negative(&'static str),
    /// Valeur hors de l'intervalle autorisé.
    OutOfRange(&'static str),
    /// Les moteurs ne peuvent pas soulever le drone (rapport poussée/poids ≤ 1).
    TooHeavy {
        /// Rapport poussée maximale / poids obtenu.
        thrust_to_weight: f64,
    },
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotFinite(n) => write!(f, "`{n}` doit être un nombre fini"),
            Self::NotPositive(n) => write!(f, "`{n}` doit être strictement positif"),
            Self::Negative(n) => write!(f, "`{n}` ne peut pas être négatif"),
            Self::OutOfRange(n) => write!(f, "`{n}` est hors de l'intervalle autorisé"),
            Self::TooHeavy { thrust_to_weight } => write!(
                f,
                "le drone ne peut pas décoller : rapport poussée/poids = {thrust_to_weight:.2} (il faut > 1)"
            ),
        }
    }
}

fn finite(name: &'static str, v: f64) -> Result<f64, ConfigError> {
    if v.is_finite() {
        Ok(v)
    } else {
        Err(ConfigError::NotFinite(name))
    }
}

fn positive(name: &'static str, v: f64) -> Result<f64, ConfigError> {
    finite(name, v)?;
    if v <= 0.0 {
        Err(ConfigError::NotPositive(name))
    } else {
        Ok(v)
    }
}

fn non_negative(name: &'static str, v: f64) -> Result<f64, ConfigError> {
    finite(name, v)?;
    if v < 0.0 {
        Err(ConfigError::Negative(name))
    } else {
        Ok(v)
    }
}

/// Valeur dans `]0, 1]` si `include_zero` est faux, `[0, 1]` sinon.
fn unit(name: &'static str, v: f64, include_zero: bool) -> Result<f64, ConfigError> {
    finite(name, v)?;
    let low_ok = if include_zero { v >= 0.0 } else { v > 0.0 };
    if low_ok && v <= 1.0 {
        Ok(v)
    } else {
        Err(ConfigError::OutOfRange(name))
    }
}

fn positive_vec(name: &'static str, v: [f64; 3]) -> Result<[f64; 3], ConfigError> {
    for x in v {
        positive(name, x)?;
    }
    Ok(v)
}

fn non_negative_vec(name: &'static str, v: [f64; 3]) -> Result<[f64; 3], ConfigError> {
    for x in v {
        non_negative(name, x)?;
    }
    Ok(v)
}

/// Batterie. Les champs absents gardent le défaut (4S, 3000 mAh, 0,025 Ω, pleine).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(deny_unknown_fields, default)
)]
pub struct BatteryConfig {
    /// Nombre de cellules en série (1 à 12).
    pub cells: Option<u32>,
    /// Capacité (mAh). Si seul `cells` change, la capacité en mAh est conservée.
    pub capacity_mah: Option<f64>,
    /// Résistance interne du pack (Ω).
    pub resistance_ohm: Option<f64>,
    /// Charge initiale, dans `]0, 1]`.
    pub initial_charge: Option<f64>,
}

impl BatteryConfig {
    /// Applique les surcharges sur `base` après validation.
    pub fn apply(&self, base: Battery) -> Result<Battery, ConfigError> {
        if self.cells.is_none()
            && self.capacity_mah.is_none()
            && self.resistance_ohm.is_none()
            && self.initial_charge.is_none()
        {
            return Ok(base);
        }
        let cells = self.cells.unwrap_or(base.cells as u32);
        if !(1..=12).contains(&cells) {
            return Err(ConfigError::OutOfRange("battery.cells"));
        }
        let mah = positive("battery.capacity_mah", self.capacity_mah.unwrap_or(base.capacity_mah()))?;
        let resistance = non_negative(
            "battery.resistance_ohm",
            self.resistance_ohm.unwrap_or(base.resistance),
        )?;
        let charge = unit("battery.initial_charge", self.initial_charge.unwrap_or(1.0), false)?;
        Ok(Battery::new(cells, mah, resistance, charge))
    }
}

/// Surcharges des paramètres du drone. Les champs absents gardent le défaut.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(deny_unknown_fields, default)
)]
pub struct DroneConfig {
    /// Masse (kg). Sans `inertia_kg_m2`, les inerties sont mises à l'échelle de la masse.
    pub mass_kg: Option<f64>,
    /// Distance centre → moteur (m).
    pub arm_m: Option<f64>,
    /// Inerties diagonales `[Ixx, Iyy, Izz]` (kg·m²). Y est l'axe de lacet.
    pub inertia_kg_m2: Option<[f64; 3]>,
    /// Rapport couple de réaction / poussée d'un rotor (m).
    pub yaw_coeff_m: Option<f64>,
    /// Poussée maximale d'un moteur (N).
    pub max_motor_thrust_n: Option<f64>,
    /// Constante de temps d'un moteur (s).
    pub motor_tau_s: Option<f64>,
    /// Traînée linéaire (N·s/m).
    pub drag_n_s_m: Option<f64>,
    /// Traînée quadratique (N·s²/m²).
    pub drag_quad_n_s2_m2: Option<f64>,
    /// Rayon d'une hélice (m).
    pub rotor_radius_m: Option<f64>,
    /// Rendement global moteur + hélice, dans `]0, 1]`.
    pub rotor_efficiency: Option<f64>,
    /// Puissance de l'électronique de bord (W).
    pub avionics_power_w: Option<f64>,
    /// Rayon de la sphère de collision (m).
    pub collision_radius_m: Option<f64>,
    /// Coefficient de restitution des chocs, dans `[0, 1]`.
    pub restitution: Option<f64>,
    /// Vitesse d'impact au-delà de laquelle le drone est détruit (m/s).
    pub crash_speed_m_s: Option<f64>,
    /// Batterie.
    pub battery: BatteryConfig,
}

impl DroneConfig {
    /// Applique les surcharges sur `base` après validation.
    pub fn apply(&self, base: QuadParams) -> Result<QuadParams, ConfigError> {
        let mut p = base;

        if let Some(m) = self.mass_kg {
            let m = positive("mass_kg", m)?;
            if self.inertia_kg_m2.is_none() {
                let k = m / base.mass;
                for i in p.inertia.iter_mut() {
                    *i *= k;
                }
            }
            p.mass = m;
        }
        if let Some(v) = self.arm_m {
            p.arm = positive("arm_m", v)?;
        }
        if let Some(i) = self.inertia_kg_m2 {
            p.inertia = positive_vec("inertia_kg_m2", i)?;
        }
        if let Some(v) = self.yaw_coeff_m {
            p.yaw_coeff = positive("yaw_coeff_m", v)?;
        }
        if let Some(v) = self.max_motor_thrust_n {
            p.max_motor_thrust = positive("max_motor_thrust_n", v)?;
        }
        if let Some(v) = self.motor_tau_s {
            p.motor_tau = positive("motor_tau_s", v)?;
        }
        if let Some(v) = self.drag_n_s_m {
            p.drag = non_negative("drag_n_s_m", v)?;
        }
        if let Some(v) = self.drag_quad_n_s2_m2 {
            p.drag_quad = non_negative("drag_quad_n_s2_m2", v)?;
        }
        if let Some(v) = self.rotor_radius_m {
            p.rotor_radius = positive("rotor_radius_m", v)?;
        }
        if let Some(v) = self.rotor_efficiency {
            p.rotor_efficiency = unit("rotor_efficiency", v, false)?;
        }
        if let Some(v) = self.avionics_power_w {
            p.avionics_power = non_negative("avionics_power_w", v)?;
        }
        if let Some(v) = self.collision_radius_m {
            p.collision_radius = positive("collision_radius_m", v)?;
        }
        if let Some(v) = self.restitution {
            p.restitution = unit("restitution", v, true)?;
        }
        if let Some(v) = self.crash_speed_m_s {
            p.crash_speed = positive("crash_speed_m_s", v)?;
        }
        p.battery = self.battery.apply(base.battery)?;

        let twr = 4.0 * p.max_motor_thrust / p.weight();
        if twr <= 1.0 {
            return Err(ConfigError::TooHeavy { thrust_to_weight: twr });
        }
        Ok(p)
    }
}

/// Surcharges des gains du contrôleur. Les champs absents gardent le défaut.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(deny_unknown_fields, default)
)]
pub struct GainsConfig {
    /// Position → vitesse désirée, par axe monde `[x, y, z]` (1/s).
    pub pos_kp: Option<[f64; 3]>,
    /// Vitesse → accélération désirée, par axe monde (1/s).
    pub vel_kp: Option<[f64; 3]>,
    /// Intégrale de l'erreur de vitesse, par axe monde (1/s²).
    pub vel_ki: Option<[f64; 3]>,
    /// Borne de l'intégrale (m/s²).
    pub max_int: Option<f64>,
    /// Erreur d'attitude → accélération angulaire, par axe corps (1/s²).
    pub att_kp: Option<[f64; 3]>,
    /// Amortissement angulaire, par axe corps (1/s).
    pub att_kd: Option<[f64; 3]>,
    /// Erreur d'attitude maximale prise en compte (rad, ≤ π).
    pub max_att_err: Option<f64>,
    /// Vitesse horizontale maximale (m/s).
    pub max_speed_h: Option<f64>,
    /// Vitesse de montée maximale (m/s).
    pub max_speed_up: Option<f64>,
    /// Vitesse de descente maximale (m/s).
    pub max_speed_down: Option<f64>,
    /// Accélération horizontale maximale (m/s²).
    pub max_accel_h: Option<f64>,
    /// Accélération verticale maximale (m/s²).
    pub max_accel_v: Option<f64>,
    /// Décélération du contrôleur en racine carrée, horizontal (m/s²).
    pub pos_accel_h: Option<f64>,
    /// Décélération du contrôleur en racine carrée, vertical (m/s²).
    pub pos_accel_v: Option<f64>,
    /// Inclinaison maximale (rad, ≤ 1,2).
    pub max_tilt: Option<f64>,
}

impl GainsConfig {
    /// Applique les surcharges sur `base` après validation.
    pub fn apply(&self, base: Gains) -> Result<Gains, ConfigError> {
        let mut g = base;
        if let Some(v) = self.pos_kp {
            g.pos_kp = positive_vec("gains.pos_kp", v)?;
        }
        if let Some(v) = self.vel_kp {
            g.vel_kp = positive_vec("gains.vel_kp", v)?;
        }
        if let Some(v) = self.vel_ki {
            g.vel_ki = non_negative_vec("gains.vel_ki", v)?;
        }
        if let Some(v) = self.max_int {
            g.max_int = non_negative("gains.max_int", v)?;
        }
        if let Some(v) = self.att_kp {
            g.att_kp = positive_vec("gains.att_kp", v)?;
        }
        if let Some(v) = self.att_kd {
            g.att_kd = positive_vec("gains.att_kd", v)?;
        }
        if let Some(v) = self.max_att_err {
            g.max_att_err = positive("gains.max_att_err", v)?;
            if g.max_att_err > core::f64::consts::PI {
                return Err(ConfigError::OutOfRange("gains.max_att_err"));
            }
        }
        if let Some(v) = self.max_speed_h {
            g.max_speed_h = positive("gains.max_speed_h", v)?;
        }
        if let Some(v) = self.max_speed_up {
            g.max_speed_up = positive("gains.max_speed_up", v)?;
        }
        if let Some(v) = self.max_speed_down {
            g.max_speed_down = positive("gains.max_speed_down", v)?;
        }
        if let Some(v) = self.max_accel_h {
            g.max_accel_h = positive("gains.max_accel_h", v)?;
        }
        if let Some(v) = self.max_accel_v {
            g.max_accel_v = positive("gains.max_accel_v", v)?;
        }
        if let Some(v) = self.pos_accel_h {
            g.pos_accel_h = positive("gains.pos_accel_h", v)?;
        }
        if let Some(v) = self.pos_accel_v {
            g.pos_accel_v = positive("gains.pos_accel_v", v)?;
        }
        if let Some(v) = self.max_tilt {
            g.max_tilt = positive("gains.max_tilt", v)?;
            if g.max_tilt > 1.2 {
                return Err(ConfigError::OutOfRange("gains.max_tilt"));
            }
        }
        Ok(g)
    }
}

/// Vent : moyenne (repère monde, m/s), intensité des rafales (m/s), graine.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(deny_unknown_fields, default)
)]
pub struct WindConfig {
    /// Vent moyen `[x, y, z]` (m/s). Repère : +X est, −Z nord.
    pub mean_m_s: [f64; 3],
    /// Intensité des rafales (m/s).
    pub gust_m_s: f64,
    /// Graine des rafales.
    pub seed: u64,
}

impl Default for WindConfig {
    fn default() -> Self {
        Self { mean_m_s: [0.0; 3], gust_m_s: 0.0, seed: 1 }
    }
}

impl WindConfig {
    /// Valide et construit le vent.
    pub fn build(&self) -> Result<Wind, ConfigError> {
        finite("wind.mean_m_s[0]", self.mean_m_s[0])?;
        finite("wind.mean_m_s[1]", self.mean_m_s[1])?;
        finite("wind.mean_m_s[2]", self.mean_m_s[2])?;
        non_negative("wind.gust_m_s", self.gust_m_s)?;
        Ok(Wind::new(self.mean_m_s, self.gust_m_s, self.seed))
    }
}

/// Air et lieu (la pesanteur reste terrestre).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(deny_unknown_fields, default)
)]
pub struct AirConfig {
    /// Altitude du point de départ au-dessus de la mer (m). Change la densité de l'air.
    pub origin_altitude_m: Option<f64>,
}

/// Configuration complète d'un essai.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(deny_unknown_fields, default)
)]
pub struct SimConfig {
    /// Graine (capteurs).
    pub seed: u64,
    /// Drone et batterie.
    pub drone: DroneConfig,
    /// Gains du contrôleur.
    pub gains: GainsConfig,
    /// Vent.
    pub wind: WindConfig,
    /// Air.
    pub air: AirConfig,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            seed: 1,
            drone: DroneConfig::default(),
            gains: GainsConfig::default(),
            wind: WindConfig::default(),
            air: AirConfig::default(),
        }
    }
}

impl SimConfig {
    /// Paramètres du drone après surcharges et validation.
    pub fn params(&self) -> Result<QuadParams, ConfigError> {
        self.drone.apply(QuadParams::default())
    }

    /// Gains après surcharges et validation.
    pub fn gains(&self) -> Result<Gains, ConfigError> {
        self.gains.apply(Gains::default())
    }

    /// Valide la configuration et construit la simulation.
    pub fn build(&self) -> Result<Simulation, ConfigError> {
        let params = self.params()?;
        let gains = self.gains()?;
        let wind = self.wind.build()?;
        let mut sim = Simulation::new(params, gains, self.seed);
        sim.env.wind = wind;
        if let Some(a) = self.air.origin_altitude_m {
            sim.env.origin_altitude = finite("air.origin_altitude_m", a)?;
        }
        Ok(sim)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config_builds() {
        assert!(SimConfig::default().build().is_ok());
    }

    #[test]
    fn empty_override_keeps_defaults() {
        let base = QuadParams::default();
        assert_eq!(DroneConfig::default().apply(base).unwrap(), base);
        assert_eq!(GainsConfig::default().apply(Gains::default()).unwrap(), Gains::default());
    }

    #[test]
    fn heavier_drone_scales_inertia() {
        let base = QuadParams::default();
        let cfg = DroneConfig { mass_kg: Some(base.mass * 1.2), ..Default::default() };
        let p = cfg.apply(base).unwrap();
        assert!((p.inertia[0] / base.inertia[0] - 1.2).abs() < 1e-12);
        assert!((p.inertia[1] / base.inertia[1] - 1.2).abs() < 1e-12);
    }

    #[test]
    fn explicit_inertia_is_not_scaled() {
        let base = QuadParams::default();
        let cfg = DroneConfig {
            mass_kg: Some(1.5),
            inertia_kg_m2: Some([0.01, 0.02, 0.01]),
            ..Default::default()
        };
        assert_eq!(cfg.apply(base).unwrap().inertia, [0.01, 0.02, 0.01]);
    }

    #[test]
    fn rejects_bad_values() {
        let base = QuadParams::default();
        let neg = DroneConfig { mass_kg: Some(-1.0), ..Default::default() };
        assert_eq!(neg.apply(base), Err(ConfigError::NotPositive("mass_kg")));
        let nan = DroneConfig { arm_m: Some(f64::NAN), ..Default::default() };
        assert_eq!(nan.apply(base), Err(ConfigError::NotFinite("arm_m")));
        let eff = DroneConfig { rotor_efficiency: Some(1.5), ..Default::default() };
        assert_eq!(eff.apply(base), Err(ConfigError::OutOfRange("rotor_efficiency")));
        let huge = DroneConfig { mass_kg: Some(100.0), ..Default::default() };
        assert!(matches!(huge.apply(base), Err(ConfigError::TooHeavy { .. })));
    }

    #[test]
    fn rejects_negative_gust() {
        let w = WindConfig { gust_m_s: -1.0, ..Default::default() };
        assert!(w.build().is_err());
    }

    #[test]
    fn battery_config_changes_pack() {
        let cfg = BatteryConfig {
            cells: Some(6),
            capacity_mah: Some(5000.0),
            initial_charge: Some(0.5),
            ..Default::default()
        };
        let b = cfg.apply(Battery::default()).unwrap();
        assert_eq!(b.cells, 6.0);
        assert!((b.capacity_mah() - 5000.0).abs() < 1e-6);
        assert!((b.soc() - 0.5).abs() < 1e-12);
    }

    #[test]
    fn only_cells_keeps_capacity_mah() {
        let cfg = BatteryConfig { cells: Some(6), ..Default::default() };
        let b = cfg.apply(Battery::default()).unwrap();
        assert!((b.capacity_mah() - 3000.0).abs() < 1e-6);
    }

    #[test]
    fn rejects_bad_battery() {
        let base = Battery::default();
        let c = BatteryConfig { cells: Some(0), ..Default::default() };
        assert_eq!(c.apply(base), Err(ConfigError::OutOfRange("battery.cells")));
        let c = BatteryConfig { initial_charge: Some(0.0), ..Default::default() };
        assert!(c.apply(base).is_err());
        let c = BatteryConfig { capacity_mah: Some(-5.0), ..Default::default() };
        assert!(c.apply(base).is_err());
    }

    #[test]
    fn gains_config_overrides_and_validates() {
        let c = GainsConfig { max_speed_h: Some(3.0), ..Default::default() };
        assert_eq!(c.apply(Gains::default()).unwrap().max_speed_h, 3.0);
        let c = GainsConfig { att_kp: Some([100.0, 0.0, 100.0]), ..Default::default() };
        assert!(c.apply(Gains::default()).is_err());
        let c = GainsConfig { max_tilt: Some(2.0), ..Default::default() };
        assert_eq!(
            c.apply(Gains::default()),
            Err(ConfigError::OutOfRange("gains.max_tilt"))
        );
    }

    #[test]
    fn battery_survives_reset() {
        let cfg = SimConfig {
            drone: DroneConfig {
                battery: BatteryConfig { cells: Some(6), ..Default::default() },
                ..Default::default()
            },
            ..Default::default()
        };
        let mut sim = cfg.build().unwrap();
        sim.reset();
        assert_eq!(sim.quad.state.battery.cells, 6.0);
    }

    #[test]
    fn heavy_but_flyable_drone_takes_off() {
        let cfg = SimConfig {
            drone: DroneConfig { mass_kg: Some(1.5), ..Default::default() },
            ..Default::default()
        };
        let mut sim = cfg.build().unwrap();
        let st = sim.quad.state;
        sim.autopilot
            .command(crate::Command::Takeoff { altitude: 2.0 }, &st);
        for _ in 0..(8.0 * 240.0) as usize {
            sim.step(1.0 / 240.0);
        }
        assert!((sim.quad.state.altitude() - 2.0).abs() < 0.1);
    }
}