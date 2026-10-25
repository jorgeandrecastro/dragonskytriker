use crate::control::Setpoint;
use crate::dynamics::QuadState;
use tiny_rodriguez_nostd::linalg::{add, dot, norm, scale, sub};
use tiny_rodriguez_nostd::Vec3;

/// Point de passage.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Waypoint {
    /// Position (m).
    pub position: Vec3,
    /// Cap à adopter (rad).
    pub yaw: f64,
    /// Temps de maintien une fois atteint (s). 0 = simple survol.
    pub hold: f64,
}

impl Waypoint {
    /// Point de passage sans maintien, cap 0.
    pub const fn at(x: f64, y: f64, z: f64) -> Self {
        Self {
            position: [x, y, z],
            yaw: 0.0,
            hold: 0.0,
        }
    }

    /// Même point avec un cap donné.
    pub const fn with_yaw(mut self, yaw: f64) -> Self {
        self.yaw = yaw;
        self
    }

    /// Même point avec un temps de maintien.
    pub const fn with_hold(mut self, hold: f64) -> Self {
        self.hold = hold;
        self
    }
}

/// Mission de capacité fixe `N` (aucune allocation).
///
/// La consigne produite est un « point lièvre » placé devant le drone sur
/// la route vers le prochain point : le mouvement est continu et fluide.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mission<const N: usize> {
    points: [Waypoint; N],
    len: usize,
    index: usize,
    hold_timer: f64,
    finished: bool,
    /// Rayon de capture d'un point de passage (m).
    pub radius: f64,
    /// Distance d'anticipation du point lièvre (m).
    pub lookahead: f64,
    /// Recommencer à la fin.
    pub looping: bool,
}

impl<const N: usize> Default for Mission<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Mission<N> {
    /// Mission vide.
    pub const fn new() -> Self {
        Self {
            points: [Waypoint::at(0.0, 0.0, 0.0); N],
            len: 0,
            index: 0,
            hold_timer: 0.0,
            finished: false,
            radius: 0.8,
            lookahead: 2.5,
            looping: false,
        }
    }

    /// Ajoute un point. Rend le point si la mission est pleine.
    pub fn push(&mut self, wp: Waypoint) -> Result<(), Waypoint> {
        if self.len == N {
            return Err(wp);
        }
        self.points[self.len] = wp;
        self.len += 1;
        Ok(())
    }

    /// Vide la mission.
    pub fn clear(&mut self) {
        self.len = 0;
        self.restart();
    }

    /// Reprend au premier point.
    pub fn restart(&mut self) {
        self.index = 0;
        self.hold_timer = 0.0;
        self.finished = false;
    }

    /// Nombre de points.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Vrai si la mission est vide.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Les points de passage.
    pub fn waypoints(&self) -> &[Waypoint] {
        &self.points[..self.len]
    }

    /// Indice du point visé.
    pub fn current_index(&self) -> usize {
        self.index
    }

    /// Point visé, s'il existe.
    pub fn current(&self) -> Option<&Waypoint> {
        if self.index < self.len {
            Some(&self.points[self.index])
        } else {
            None
        }
    }

    /// Vrai quand tous les points ont été atteints (jamais en mode boucle).
    pub fn is_finished(&self) -> bool {
        self.finished
    }

    /// Indice du point suivant, s'il existe.
    fn next_index(&self) -> Option<usize> {
        if self.index + 1 < self.len {
            Some(self.index + 1)
        } else if self.looping && self.len > 1 {
            Some(0)
        } else {
            None
        }
    }

    /// Met à jour la mission et retourne la consigne, ou `None` si vide.
    ///
    /// Un point est validé quand le drone passe dans son rayon de capture, ou, pour un
    /// simple survol (`hold == 0`), quand il l'a franchi en allant vers le suivant.
    pub fn update(&mut self, state: &QuadState, dt: f64) -> Option<Setpoint> {
        if self.len == 0 {
            return None;
        }
        let pos = state.position();
        let wp = self.points[self.index];
        let next = self.next_index();
        let to = sub(&wp.position, &pos);
        let dist = norm(&to);

        if !self.finished {
            let mut advance = false;
            if dist < self.radius {
                self.hold_timer += dt;
                advance = self.hold_timer >= wp.hold;
            } else if wp.hold <= 0.0 {
                // Franchissement : le drone est passé de l'autre côté du point, tout près.
                if let Some(n) = next {
                    let leg = sub(&self.points[n].position, &wp.position);
                    let passed = dot(&sub(&pos, &wp.position), &leg) > 0.0;
                    advance = passed && dist < 2.0 * self.radius;
                }
            }
            if advance {
                self.hold_timer = 0.0;
                match next {
                    Some(n) => self.index = n,
                    None => self.finished = true,
                }
            }
        }

        // Point lièvre : devant le drone sur la route du point visé.
        let wp = self.points[self.index];
        let next = self.next_index();
        let to = sub(&wp.position, &pos);
        let dist = norm(&to);
        let position = if dist > self.lookahead {
            add(&pos, &scale(&to, self.lookahead / dist))
        } else if let (Some(n), true) = (next, wp.hold <= 0.0) {
            // Près d'un survol : on vise au-delà, à distance constante, pour enchaîner sans ralentir.
            let leg = sub(&self.points[n].position, &wp.position);
            let len = norm(&leg);
            if len > 1e-9 {
                add(&wp.position, &scale(&leg, self.lookahead / len))
            } else {
                wp.position
            }
        } else {
            wp.position
        };
        Some(Setpoint { position, velocity: [0.0; 3], yaw: wp.yaw })
    }
}