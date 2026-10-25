use crate::battery::Battery;
use crate::GRAVITY;

/// Paramètres physiques d'un quadricoptère en X.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuadParams {
    /// Masse (kg).
    pub mass: f64,
    /// Distance centre → moteur (m).
    pub arm: f64,
    /// Inertie diagonale `[Ixx, Iyy, Izz]` (kg·m²). Y est l'axe de lacet.
    pub inertia: [f64; 3],
    /// Rapport couple de réaction / poussée d'un rotor (m).
    pub yaw_coeff: f64,
    /// Poussée maximale d'un moteur (N).
    pub max_motor_thrust: f64,
    /// Constante de temps d'un moteur (s).
    pub motor_tau: f64,
    /// Traînée linéaire (N·s/m).
    pub drag: f64,
    /// Traînée quadratique (N·s²/m²).
    pub drag_quad: f64,
    /// Rayon d'une hélice (m) : effet de sol et puissance.
    pub rotor_radius: f64,
    /// Rendement global moteur + hélice (0 à 1).
    pub rotor_efficiency: f64,
    /// Puissance de l'électronique de bord (W).
    pub avionics_power: f64,
    /// Rayon de la sphère de collision (m).
    pub collision_radius: f64,
    /// Coefficient de restitution des chocs (0 à 1).
    pub restitution: f64,
    /// Vitesse d'impact au-delà de laquelle le drone est détruit (m/s).
    pub crash_speed: f64,
    /// Batterie (état initial à chaque décollage depuis un reset).
    pub battery: Battery,
}

impl Default for QuadParams {
    /// Le « Dragon X1 » : 0,9 kg, rapport poussée/poids ≈ 3,6.
    fn default() -> Self {
        Self {
            mass: 0.9,
            arm: 0.18,
            inertia: [0.0075, 0.0140, 0.0075],
            yaw_coeff: 0.016,
            max_motor_thrust: 8.0,
            motor_tau: 0.03,
            drag: 0.08,
            drag_quad: 0.03,
            rotor_radius: 0.09,
            rotor_efficiency: 0.55,
            avionics_power: 6.0,
            collision_radius: 0.17,
            restitution: 0.25,
            crash_speed: 5.0,
            battery: Battery::default(),
        }
    }
}

impl QuadParams {
    /// Poids (N).
    #[inline]
    pub fn weight(&self) -> f64 {
        self.mass * GRAVITY
    }

    /// Poussée par moteur en vol stationnaire (N).
    #[inline]
    pub fn hover_thrust_per_motor(&self) -> f64 {
        self.weight() / 4.0
    }

    /// Distance de chaque moteur à l'axe, projetée sur X et Z (m).
    #[inline]
    pub fn arm_xz(&self) -> f64 {
        self.arm * core::f64::consts::FRAC_1_SQRT_2
    }
}