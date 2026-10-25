use crate::battery::Battery;
use crate::mixer::{clamp, Mixer};
use crate::params::QuadParams;
use crate::world::{Collider, EmptyWorld};
use tiny_rodriguez_nostd::linalg::{add, cross, dot, norm, scale};
use tiny_rodriguez_nostd::{Pose3, Rotation3, Vec3};

/// Air ambiant vu par le drone.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Air {
    /// Vitesse du vent, repère monde (m/s).
    pub wind: Vec3,
    /// Masse volumique (kg/m³).
    pub density: f64,
}

impl Air {
    /// Air calme au niveau de la mer.
    pub const CALM: Air = Air { wind: [0.0; 3], density: 1.225 };
}

/// État complet du quadricoptère.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuadState {
    /// Pose dans le monde (rotation corps → monde, position).
    pub pose: Pose3,
    /// Vitesse linéaire, repère **monde** (m/s).
    pub velocity: Vec3,
    /// Vitesse angulaire, repère **corps** (rad/s).
    pub omega: Vec3,
    /// Poussée actuelle de chaque moteur (N), après la dynamique du moteur.
    pub motors: [f64; 4],
    /// Vrai quand le drone repose au sol.
    pub on_ground: bool,
    /// Batterie.
    pub battery: Battery,
    /// Force spécifique (accélération non gravitationnelle) dans le repère **corps** (m/s²).
    pub specific_force: Vec3,
    /// Vrai si le drone est détruit (impact trop violent).
    pub crashed: bool,
    /// Vitesse d'impact la plus forte depuis le dernier reset (m/s).
    pub peak_impact: f64,
    /// Puissance électrique actuelle (W).
    pub power: f64,
}

impl Default for QuadState {
    fn default() -> Self {
        Self {
            pose: Pose3::identity(),
            velocity: [0.0; 3],
            omega: [0.0; 3],
            motors: [0.0; 4],
            on_ground: true,
            battery: Battery::default(),
            specific_force: [0.0, 9.81, 0.0],
            crashed: false,
            peak_impact: 0.0,
            power: 0.0,
        }
    }
}

impl QuadState {
    /// Position (m).
    #[inline]
    pub fn position(&self) -> Vec3 {
        self.pose.translation
    }

    /// Altitude (m).
    #[inline]
    pub fn altitude(&self) -> f64 {
        self.pose.translation[1]
    }

    /// Vitesse scalaire (m/s).
    #[inline]
    pub fn speed(&self) -> f64 {
        norm(&self.velocity)
    }

    /// Axe +Y du corps exprimé dans le monde (direction de la poussée).
    #[inline]
    pub fn thrust_axis(&self) -> Vec3 {
        self.pose.rotation.rotate(&[0.0, 1.0, 0.0])
    }

    /// Inclinaison par rapport à la verticale (rad).
    #[inline]
    pub fn tilt(&self) -> f64 {
        let c = self.thrust_axis()[1];
        libm::acos(clamp(c, -1.0, 1.0))
    }

    /// Lacet (rad) : cap du drone autour de +Y, 0 = vers −Z, positif vers −X.
    #[inline]
    pub fn yaw(&self) -> f64 {
        yaw_of(&self.pose.rotation)
    }
}

/// Lacet d'une rotation (voir [`QuadState::yaw`]).
pub fn yaw_of(r: &Rotation3) -> f64 {
    let f = r.rotate(&[0.0, 0.0, -1.0]);
    libm::atan2(-f[0], -f[2])
}

/// Quadricoptère simulé : paramètres + état.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quadrotor {
    /// Paramètres physiques.
    pub params: QuadParams,
    /// État courant.
    pub state: QuadState,
    mixer: Mixer,
}

impl Quadrotor {
    /// Crée un drone posé à l'origine, avec la batterie décrite par `params.battery`.
    pub fn new(params: QuadParams) -> Self {
        let state = QuadState { battery: params.battery, ..QuadState::default() };
        Self { mixer: Mixer::new(&params), params, state }
    }

    /// Replace le drone au sol à `position` (y est ramené à 0), cap `yaw`,
    /// avec la batterie initiale décrite par `params.battery`.
    pub fn reset(&mut self, position: Vec3, yaw: f64) {
        self.state = QuadState {
            pose: Pose3::new(
                Rotation3::from_axis_angle(&[0.0, 1.0, 0.0], yaw),
                [position[0], 0.0, position[2]],
            ),
            battery: self.params.battery,
            ..QuadState::default()
        };
    }

    /// Mixeur associé à ce drone.
    #[inline]
    pub fn mixer(&self) -> &Mixer {
        &self.mixer
    }

    /// Avance dans un air calme, sans obstacle.
    pub fn step(&mut self, motor_cmd: &[f64; 4], dt: f64) {
        self.step_with(motor_cmd, dt, &Air::CALM, &EmptyWorld, 9.81);
    }

    /// Avance de `dt` avec poussées commandées (N), air, monde et pesanteur.
    pub fn step_with<C: Collider>(
        &mut self,
        motor_cmd: &[f64; 4],
        dt: f64,
        air: &Air,
        world: &C,
        gravity: f64,
    ) {
        let p = self.params;
        let s = &mut self.state;
        let v_before = s.velocity;

        // 1. Moteurs : réponse du 1er ordre, bornée par la tension de la batterie.
        let dead = s.crashed || s.battery.is_empty();
        let limit = p.max_motor_thrust * s.battery.thrust_factor();
        let alpha = dt / (p.motor_tau + dt);
        for i in 0..4 {
            let target = if dead { 0.0 } else { clamp(motor_cmd[i], 0.0, limit) };
            s.motors[i] += (target - s.motors[i]) * alpha;
        }

        // 2. Poussée réelle : densité de l'air et effet de sol (Cheeseman-Bennett).
        let h = s.pose.translation[1].max(p.rotor_radius);
        let x = p.rotor_radius / (4.0 * h);
        let ground_effect = 1.0 / (1.0 - x * x);
        let k = ground_effect * (air.density / 1.225);
        let eff = [s.motors[0] * k, s.motors[1] * k, s.motors[2] * k, s.motors[3] * k];
        let (thrust, torque) = self.mixer.wrench(&eff);

        // 3. Puissance électrique (théorie du disque) → batterie.
        let disc = core::f64::consts::PI * p.rotor_radius * p.rotor_radius;
        let denom = libm::sqrt(2.0 * air.density * disc) * p.rotor_efficiency;
        let mut power = p.avionics_power;
        for f in s.motors {
            power += f * libm::sqrt(f) / denom;
        }
        s.power = power;
        s.battery.step(power, dt);

        // 4. Rotation (équation d'Euler, inertie diagonale).
        let i = p.inertia;
        let iw = [i[0] * s.omega[0], i[1] * s.omega[1], i[2] * s.omega[2]];
        let gyro = cross(&s.omega, &iw);
        for a in 0..3 {
            s.omega[a] += (torque[a] - gyro[a]) / i[a] * dt;
        }

        // 5. Translation : poussée + traînée par rapport à l'air − poids.
        let thrust_world = s.pose.rotation.rotate(&[0.0, thrust, 0.0]);
        let v_rel = [
            s.velocity[0] - air.wind[0],
            s.velocity[1] - air.wind[1],
            s.velocity[2] - air.wind[2],
        ];
        let c = p.drag + p.drag_quad * norm(&v_rel);
        for a in 0..3 {
            let mut f = thrust_world[a] - c * v_rel[a];
            if a == 1 {
                f -= p.mass * gravity;
            }
            s.velocity[a] += f / p.mass * dt;
        }

        // 6. Sol : posé tant que la poussée ne soulève pas le drone.
        let grounded =
            s.pose.translation[1] <= 0.0 && s.velocity[1] <= 0.0 && thrust < p.mass * gravity;
        if grounded {
            if s.velocity[1] < -p.crash_speed {
                s.crashed = true;
            }
            if -v_before[1] > s.peak_impact && v_before[1] < -0.5 {
                s.peak_impact = -v_before[1];
            }
            s.on_ground = true;
            s.pose.translation[1] = 0.0;
            s.velocity = [0.0; 3];
            s.omega = [0.0; 3];
            let yaw = yaw_of(&s.pose.rotation);
            s.pose.rotation = Rotation3::from_axis_angle(&[0.0, 1.0, 0.0], yaw);
        } else {
            s.on_ground = false;

            // 7. Intégration exacte sur SE(3) d'un torseur corps constant sur dt.
            let v_body = s.pose.rotation.inverse().rotate(&s.velocity);
            let twist = [v_body[0], v_body[1], v_body[2], s.omega[0], s.omega[1], s.omega[2]];
            s.pose = s.pose.integrate(&twist, dt);

            // 8. Collisions avec les obstacles (sphère englobante).
            for _ in 0..3 {
                let Some(ct) = world.contact(&s.pose.translation, p.collision_radius) else {
                    break;
                };
                s.pose.translation = add(&s.pose.translation, &scale(&ct.normal, ct.depth));
                let vn = dot(&s.velocity, &ct.normal);
                if vn < 0.0 {
                    let impact = -vn;
                    s.velocity = add(&s.velocity, &scale(&ct.normal, -(1.0 + p.restitution) * vn));
                    s.velocity = scale(&s.velocity, 0.96);
                    let n_body = s.pose.rotation.inverse().rotate(&ct.normal);
                    let kick = cross(&n_body, &[0.0, 1.0, 0.0]);
                    for a in 0..3 {
                        s.omega[a] += 2.0 * impact * kick[a] * (0.0075 / p.inertia[a]);
                    }
                    if impact > s.peak_impact {
                        s.peak_impact = impact;
                    }
                    if impact > p.crash_speed {
                        s.crashed = true;
                    }
                }
            }

            // Garde-fou : le sol est dur.
            if s.pose.translation[1] < 0.0 {
                s.pose.translation[1] = 0.0;
                if s.velocity[1] < -p.crash_speed {
                    s.crashed = true;
                }
                if s.velocity[1] < 0.0 {
                    s.velocity[1] = 0.0;
                }
            }

            // Seuil assoupli à 1e-6 pour stopper le clignotement graphique.
            if s.pose.rotation.orthogonality_error() > 1e-6 {
                s.pose.rotation = s.pose.rotation.renormalized();
            }
        }

        // 9. Force spécifique.
        let acc = [
            (s.velocity[0] - v_before[0]) / dt,
            (s.velocity[1] - v_before[1]) / dt + gravity,
            (s.velocity[2] - v_before[2]) / dt,
        ];
        s.specific_force = s.pose.rotation.inverse().rotate(&acc);
    }
}