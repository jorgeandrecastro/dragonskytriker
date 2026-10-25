use crate::dynamics::QuadState;
use crate::mixer::{clamp, Mixer};
use crate::params::QuadParams;
use crate::GRAVITY;
use tiny_rodriguez_nostd::linalg::{cross, dot, norm, scale};
use tiny_rodriguez_nostd::{Rotation3, Vec3};

/// Gains et limites du contrôleur.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gains {
    /// Position → vitesse désirée (1/s). Horizontal : `[0]` sert pour x et z.
    pub pos_kp: Vec3,
    /// Vitesse → accélération désirée (1/s).
    pub vel_kp: Vec3,
    /// Intégrale de l'erreur de vitesse (1/s²).
    pub vel_ki: Vec3,
    /// Borne de l'intégrale (m/s²).
    pub max_int: f64,
    /// Erreur d'attitude → accélération angulaire, par axe corps (1/s²).
    pub att_kp: Vec3,
    /// Amortissement angulaire, par axe corps (1/s).
    pub att_kd: Vec3,
    /// Erreur d'attitude maximale prise en compte (rad).
    pub max_att_err: f64,
    /// Vitesse horizontale maximale (m/s).
    pub max_speed_h: f64,
    /// Vitesse de montée maximale (m/s).
    pub max_speed_up: f64,
    /// Vitesse de descente maximale (m/s).
    pub max_speed_down: f64,
    /// Accélération horizontale maximale (m/s²).
    pub max_accel_h: f64,
    /// Accélération verticale maximale (m/s²).
    pub max_accel_v: f64,
    /// Décélération du contrôleur en racine carrée, horizontal (m/s²).
    pub pos_accel_h: f64,
    /// Décélération du contrôleur en racine carrée, vertical (m/s²).
    pub pos_accel_v: f64,
    /// Inclinaison maximale (rad).
    pub max_tilt: f64,
}

impl Default for Gains {
    fn default() -> Self {
        Self {
            pos_kp: [0.9, 1.5, 0.9],
            vel_kp: [2.6, 5.0, 2.6],
            vel_ki: [0.8, 1.5, 0.8],
            max_int: 2.5,
            att_kp: [150.0, 30.0, 150.0],
            att_kd: [24.0, 10.0, 24.0],
            max_att_err: 0.8,
            max_speed_h: 5.0,
            max_speed_up: 3.0,
            max_speed_down: 1.8,
            max_accel_h: 4.0,
            max_accel_v: 5.0,
            pos_accel_h: 2.2,
            pos_accel_v: 2.0,
            max_tilt: 0.52, // ≈ 30°
        }
    }
}

/// Consigne de vol.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Setpoint {
    /// Position désirée (m).
    pub position: Vec3,
    /// Vitesse désirée en anticipation (m/s).
    pub velocity: Vec3,
    /// Lacet désiré (rad).
    pub yaw: f64,
}

impl Setpoint {
    /// Tenir `position` avec le cap `yaw`.
    pub fn hover_at(position: Vec3, yaw: f64) -> Self {
        Self {
            position,
            velocity: [0.0; 3],
            yaw,
        }
    }
}

/// Erreur → vitesse désirée : linéaire près de la cible, racine carrée au loin
/// (décélération constante `a_max`, donc pas de dépassement).
fn sqrt_ctrl(err: f64, kp: f64, a_max: f64) -> f64 {
    let lin = a_max / (kp * kp);
    if err.abs() <= lin {
        kp * err
    } else {
        err.signum() * libm::sqrt(2.0 * a_max * (err.abs() - 0.5 * lin))
    }
}

/// Contrôleur géométrique en cascade.
///
/// 1. position (racine carrée) → vitesse → accélération désirée (monde) ;
/// 2. intégrale de vitesse avec anti-windup (vent, erreurs de modèle) ;
/// 3. accélération → force → attitude désirée `R_d` (axe de poussée + cap) ;
/// 4. erreur d'attitude `log(Rᵀ R_d) ∈ so(3)` → couples (PD + terme gyroscopique) ;
/// 5. poussée + couples → 4 moteurs via le [`Mixer`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Controller {
    /// Gains et limites.
    pub gains: Gains,
    params: QuadParams,
    mixer: Mixer,
    /// Intégrale de l'erreur de vitesse (m/s²), repère monde.
    vel_int: Vec3,
}

impl Controller {
    /// Crée un contrôleur pour un drone donné.
    pub fn new(params: QuadParams, gains: Gains) -> Self {
        Self {
            gains,
            params,
            mixer: Mixer::new(&params),
            vel_int: [0.0; 3],
        }
    }

    /// Remet les intégrateurs à zéro (décollage, reset, désarmement).
    pub fn reset(&mut self) {
        self.vel_int = [0.0; 3];
    }

    /// Calcule les poussées des 4 moteurs (N).
    pub fn update(&mut self, s: &QuadState, sp: &Setpoint, dt: f64) -> [f64; 4] {
        let (rd, thrust) = self.desired_attitude_and_thrust(s, sp, dt);
        let g = self.gains;
        let p = self.params;

        // Erreur d'attitude dans le repère corps : R · exp(e) = R_d, plafonnée.
        let mut e = s.pose.rotation.inverse().compose(&rd).log();
        let en = norm(&e);
        if en > g.max_att_err {
            e = scale(&e, g.max_att_err / en);
        }
        let i = p.inertia;
        let iw = [i[0] * s.omega[0], i[1] * s.omega[1], i[2] * s.omega[2]];
        let gyro = cross(&s.omega, &iw);
        let mut tau = [0.0; 3];
        for k in 0..3 {
            let alpha = g.att_kp[k] * e[k] - g.att_kd[k] * s.omega[k];
            tau[k] = i[k] * alpha + gyro[k];
        }
        self.mixer.allocate(thrust, &tau)
    }

    /// Attitude désirée et poussée totale (N) pour une consigne.
    pub fn desired_attitude_and_thrust(
        &mut self,
        s: &QuadState,
        sp: &Setpoint,
        dt: f64,
    ) -> (Rotation3, f64) {
        let g = self.gains;
        let p = self.params;
        let pos = s.position();

        if s.on_ground {
            self.vel_int = [0.0; 3];
        }

        // 1. Position → vitesse désirée.
        let ex = sp.position[0] - pos[0];
        let ez = sp.position[2] - pos[2];
        let d = libm::hypot(ex, ez);
        let sh = if d > 1e-9 {
            sqrt_ctrl(d, g.pos_kp[0], g.pos_accel_h) / d
        } else {
            0.0
        };
        let mut vcmd = [sp.velocity[0] + ex * sh, 0.0, sp.velocity[2] + ez * sh];
        let vh = libm::hypot(vcmd[0], vcmd[2]);
        if vh > g.max_speed_h {
            let k = g.max_speed_h / vh;
            vcmd[0] *= k;
            vcmd[2] *= k;
        }
        let ey = sp.position[1] - pos[1];
        vcmd[1] = clamp(
            sp.velocity[1] + sqrt_ctrl(ey, g.pos_kp[1], g.pos_accel_v),
            -g.max_speed_down,
            g.max_speed_up,
        );

        // 2. Vitesse → accélération désirée.
        let mut a = [0.0; 3];
        let mut verr = [0.0; 3];
        for k in 0..3 {
            verr[k] = vcmd[k] - s.velocity[k];
            a[k] = g.vel_kp[k] * verr[k];
        }
        let ah = libm::hypot(a[0], a[2]);
        let sat_h = ah > g.max_accel_h;
        if sat_h {
            let k = g.max_accel_h / ah;
            a[0] *= k;
            a[2] *= k;
        }
        a[1] = clamp(a[1], -g.max_accel_v, g.max_accel_v);

        // 3. Intégrale avec anti-windup (gelée en saturation horizontale ou au sol).
        if !s.on_ground {
            for k in 0..3 {
                if sat_h && k != 1 {
                    continue;
                }
                self.vel_int[k] = clamp(
                    self.vel_int[k] + g.vel_ki[k] * verr[k] * dt,
                    -g.max_int,
                    g.max_int,
                );
            }
        }

        // 4. Force désirée (monde) : masse·(a + i + g) + traînée linéaire et quadratique.
        let c = p.drag + p.drag_quad * norm(&s.velocity);
        let mut f = [
            p.mass * (a[0] + self.vel_int[0]) + c * s.velocity[0],
            p.mass * (a[1] + self.vel_int[1] + GRAVITY) + c * s.velocity[1],
            p.mass * (a[2] + self.vel_int[2]) + c * s.velocity[2],
        ];

        // 5. Limite d'inclinaison et poussée verticale minimale.
        let fy = f[1].max(0.2 * p.weight());
        let fh = libm::hypot(f[0], f[2]);
        let fh_max = libm::tan(g.max_tilt) * fy;
        if fh > fh_max {
            let k = fh_max / fh;
            f[0] *= k;
            f[2] *= k;
        }
        f[1] = fy;

        // Poussée : projection de la force sur l'axe de poussée actuel.
        let thrust = clamp(dot(&f, &s.thrust_axis()), 0.0, 4.0 * p.max_motor_thrust);

        // 6. Attitude désirée : +Y = direction de la force, cap imposé par le lacet.
        let by = scale(&f, 1.0 / norm(&f));
        let zc = [libm::sin(sp.yaw), 0.0, libm::cos(sp.yaw)]; // « arrière » du cap
        let bx_raw = cross(&by, &zc);
        let bx = scale(&bx_raw, 1.0 / norm(&bx_raw));
        let bz = cross(&bx, &by);
        let rd = Rotation3 {
            matrix: [
                [bx[0], by[0], bz[0]],
                [bx[1], by[1], bz[1]],
                [bx[2], by[2], bz[2]],
            ],
        };
        (rd, thrust)
    }
}