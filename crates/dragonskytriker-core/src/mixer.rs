use crate::params::QuadParams;
use tiny_rodriguez_nostd::Vec3;

/// Ordre des moteurs : 0 avant-droit, 1 avant-gauche, 2 arrière-droit, 3 arrière-gauche.
/// (L'avant est −Z, la droite est +X.)
const X_SIGN: [f64; 4] = [1.0, -1.0, 1.0, -1.0];
const Z_SIGN: [f64; 4] = [-1.0, -1.0, 1.0, 1.0];
/// Signe du couple de réaction de chaque rotor sur le corps, autour de +Y.
const SPIN: [f64; 4] = [-1.0, 1.0, 1.0, -1.0];

/// Mixeur d'un quadricoptère en X.
///
/// Relie la poussée totale `T` et les couples `[τx, τy, τz]` aux poussées des
/// quatre moteurs. La matrice étant orthogonale, l'inverse est direct.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mixer {
    d: f64,
    k: f64,
    max_thrust: f64,
}

impl Mixer {
    /// Construit le mixeur à partir des paramètres du drone.
    pub fn new(p: &QuadParams) -> Self {
        Self {
            d: p.arm_xz(),
            k: p.yaw_coeff,
            max_thrust: p.max_motor_thrust,
        }
    }

    /// Poussées des moteurs (N) pour une poussée totale et des couples corps.
    /// Chaque moteur est borné à `[0, max_motor_thrust]`.
    pub fn allocate(&self, thrust: f64, torque: &Vec3) -> [f64; 4] {
        let mut f = [0.0; 4];
        for i in 0..4 {
            let v = 0.25 * thrust
                + 0.25 * (-Z_SIGN[i] * self.d) * torque[0] / (self.d * self.d)
                + 0.25 * SPIN[i] * torque[1] / self.k
                + 0.25 * (X_SIGN[i] * self.d) * torque[2] / (self.d * self.d);
            f[i] = clamp(v, 0.0, self.max_thrust);
        }
        f
    }

    /// Poussée totale et couples produits par des poussées de moteurs données.
    pub fn wrench(&self, motors: &[f64; 4]) -> (f64, Vec3) {
        let mut t = 0.0;
        let mut tau = [0.0; 3];
        for i in 0..4 {
            t += motors[i];
            tau[0] += -Z_SIGN[i] * self.d * motors[i];
            tau[1] += SPIN[i] * self.k * motors[i];
            tau[2] += X_SIGN[i] * self.d * motors[i];
        }
        (t, tau)
    }
}

#[inline]
pub(crate) fn clamp(x: f64, lo: f64, hi: f64) -> f64 {
    if x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
    }
}
