use crate::control::{Controller, Gains, Setpoint};
use crate::dynamics::QuadState;
use crate::mission::Mission;
use crate::params::QuadParams;
use tiny_rodriguez_nostd::Vec3;

/// Mode de vol.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlightMode {
    /// Moteurs coupés.
    Disarmed,
    /// Montée verticale jusqu'à l'altitude demandée.
    Takeoff,
    /// Maintien de la consigne courante.
    Hover,
    /// Exécution de la mission.
    Mission,
    /// Descente contrôlée puis désarmement.
    Land,
    /// Consigne fournie en continu par l'application (téléopération).
    Manual,
}

/// Ordres donnés à l'autopilote.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Command {
    /// Arme les moteurs (drone au sol).
    Arm,
    /// Coupe immédiatement les moteurs.
    Disarm,
    /// Décolle verticalement jusqu'à `altitude` (arme si besoin).
    Takeoff {
        /// Altitude visée (m).
        altitude: f64,
    },
    /// Fige la consigne à la position actuelle.
    Hover,
    /// Rejoint un point puis le maintient.
    Goto {
        /// Position visée (m).
        position: Vec3,
        /// Cap visé (rad).
        yaw: f64,
    },
    /// Démarre la mission (décolle d'abord si nécessaire).
    StartMission,
    /// Atterrit à la verticale.
    Land,
    /// Impose une consigne (rappeler à chaque pas pour une téléopération).
    Manual(Setpoint),
}

/// Autopilote : machine à états + contrôleur + mission de capacité `N`.
#[derive(Debug, Clone, Copy)]
pub struct Autopilot<const N: usize> {
    /// Contrôleur bas niveau.
    pub controller: Controller,
    /// Mission en cours.
    pub mission: Mission<N>,
    mode: FlightMode,
    setpoint: Setpoint,
    land_height: f64,
    pending_mission: bool,
    yaw_cmd: f64,
    /// Vitesse de lacet maximale de la consigne (rad/s).
    pub max_yaw_rate: f64,
    /// Vitesse de descente à l'atterrissage (m/s).
    pub land_speed: f64,
}

impl<const N: usize> Autopilot<N> {
    /// Crée un autopilote désarmé.
    pub fn new(params: QuadParams, gains: Gains) -> Self {
        Self {
            controller: Controller::new(params, gains),
            mission: Mission::new(),
            mode: FlightMode::Disarmed,
            setpoint: Setpoint::hover_at([0.0; 3], 0.0),
            land_height: 0.0,
            pending_mission: false,
            yaw_cmd: 0.0,
            max_yaw_rate: 1.3,
            land_speed: 0.5,
        }
    }

    /// Mode de vol courant.
    pub fn mode(&self) -> FlightMode {
        self.mode
    }

    /// Consigne courante envoyée au contrôleur.
    pub fn setpoint(&self) -> &Setpoint {
        &self.setpoint
    }

    /// Exécute un ordre.
    pub fn command(&mut self, cmd: Command, state: &QuadState) {
        match cmd {
            Command::Arm => {
                if self.mode == FlightMode::Disarmed {
                    self.controller.reset();
                    self.setpoint = Setpoint::hover_at(state.position(), state.yaw());
                    self.mode = FlightMode::Hover;
                }
            }
            Command::Disarm => {
                self.controller.reset();
                self.mode = FlightMode::Disarmed;
                self.pending_mission = false;
            }
            Command::Takeoff { altitude } => {
                if self.mode == FlightMode::Disarmed {
                    self.controller.reset();
                }
                let p = state.position();
                self.setpoint = Setpoint::hover_at([p[0], altitude.max(0.3), p[2]], state.yaw());
                self.mode = FlightMode::Takeoff;
            }
            Command::Hover => {
                if self.mode != FlightMode::Disarmed {
                    self.setpoint = Setpoint::hover_at(state.position(), state.yaw());
                    self.mode = FlightMode::Hover;
                    self.pending_mission = false;
                }
            }
            Command::Goto { position, yaw } => {
                if self.mode != FlightMode::Disarmed {
                    self.setpoint = Setpoint::hover_at(
                        [position[0], position[1].max(0.0), position[2]],
                        yaw,
                    );
                    self.mode = FlightMode::Hover;
                    self.pending_mission = false;
                }
            }
            Command::StartMission => {
                if let Some(first) = self.mission.waypoints().first().copied() {
                    self.mission.restart();
                    if state.on_ground || state.altitude() < 0.3 {
                        if self.mode == FlightMode::Disarmed {
                            self.controller.reset();
                        }
                        let p = state.position();
                        self.setpoint =
                            Setpoint::hover_at([p[0], first.position[1], p[2]], state.yaw());
                        self.mode = FlightMode::Takeoff;
                        self.pending_mission = true;
                    } else {
                        self.mode = FlightMode::Mission;
                    }
                }
            }
            Command::Land => {
                if self.mode != FlightMode::Disarmed {
                    let p = state.position();
                    self.land_height = p[1];
                    self.setpoint = Setpoint::hover_at(p, state.yaw());
                    self.mode = FlightMode::Land;
                    self.pending_mission = false;
                }
            }
            Command::Manual(sp) => {
                if self.mode != FlightMode::Disarmed {
                    self.setpoint = sp;
                    self.mode = FlightMode::Manual;
                    self.pending_mission = false;
                }
            }
        }
    }

    /// Avance l'autopilote de `dt` et retourne les poussées moteurs (N).
    pub fn update(&mut self, state: &QuadState, dt: f64) -> [f64; 4] {
        match self.mode {
            FlightMode::Disarmed => {
                self.yaw_cmd = state.yaw();
                self.controller.reset();
                return [0.0; 4];
            }
            FlightMode::Takeoff => {
                let err = self.setpoint.position[1] - state.altitude();
                if libm::fabs(err) < 0.15 && libm::fabs(state.velocity[1]) < 0.3 {
                    if self.pending_mission {
                        self.pending_mission = false;
                        self.mode = FlightMode::Mission;
                    } else {
                        self.mode = FlightMode::Hover;
                    }
                }
            }
            FlightMode::Mission => {
                if let Some(sp) = self.mission.update(state, dt) {
                    self.setpoint = sp;
                }
                if self.mission.is_finished() {
                    self.setpoint = Setpoint::hover_at(
                        self.mission.current().map(|w| w.position).unwrap_or(state.position()),
                        self.setpoint.yaw,
                    );
                    self.mode = FlightMode::Hover;
                }
            }
            FlightMode::Land => {
                self.land_height = (self.land_height - self.land_speed * dt).max(-0.3);
                self.setpoint.position[1] = self.land_height;
                if state.on_ground {
                    self.controller.reset();
                    self.mode = FlightMode::Disarmed;
                    return [0.0; 4];
                }
            }
            FlightMode::Hover | FlightMode::Manual => {}
        }
        // Cap : la consigne glisse vers sa valeur à vitesse bornée (mouvement fluide).
        let mut sp = self.setpoint;
        if self.mode == FlightMode::Manual {
            self.yaw_cmd = sp.yaw;
        } else {
            let pi = core::f64::consts::PI;
            let mut d = sp.yaw - self.yaw_cmd;
            d -= 2.0 * pi * libm::floor((d + pi) / (2.0 * pi));
            let step = self.max_yaw_rate * dt;
            self.yaw_cmd += if d > step { step } else if d < -step { -step } else { d };
            sp.yaw = self.yaw_cmd;
        }
        self.controller.update(state, &sp, dt)
    }
}