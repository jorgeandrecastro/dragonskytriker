//! Le monde : obstacles, lancer de rayons (lidar, télémètre) et collisions.

use libm::{cos, fabs, sin, sqrt};
use tiny_rodriguez_nostd::linalg::{add, dot, norm, scale, sub};
use tiny_rodriguez_nostd::Vec3;

/// Forme d'obstacle. Le sol (y = 0) est toujours présent.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Shape {
    /// Sphère.
    Sphere {
        /// Centre (m).
        center: Vec3,
        /// Rayon (m).
        radius: f64,
    },
    /// Boîte tournée d'un angle `yaw` autour de Y.
    Box {
        /// Centre (m).
        center: Vec3,
        /// Demi-dimensions (m).
        half: Vec3,
        /// Rotation autour de Y (rad).
        yaw: f64,
    },
    /// Cylindre vertical.
    Cylinder {
        /// Centre de la base (m).
        base: Vec3,
        /// Rayon (m).
        radius: f64,
        /// Hauteur (m).
        height: f64,
    },
}

/// Résultat d'un lancer de rayon.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RayHit {
    /// Distance depuis l'origine (m).
    pub distance: f64,
    /// Point d'impact (monde).
    pub point: Vec3,
    /// Normale de la surface touchée (monde).
    pub normal: Vec3,
    /// Indice de l'obstacle, `None` pour le sol.
    pub obstacle: Option<usize>,
}

/// Contact entre une sphère et le monde.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Contact {
    /// Normale, de l'obstacle vers la sphère.
    pub normal: Vec3,
    /// Profondeur de pénétration (m).
    pub depth: f64,
}

/// Ce qu'un monde sait faire : répondre aux rayons et aux contacts.
pub trait Collider {
    /// Premier impact d'un rayon (`dir` unitaire), sol compris.
    fn raycast(&self, origin: &Vec3, dir: &Vec3, max_distance: f64) -> Option<RayHit>;
    /// Contact le plus profond d'une sphère avec les obstacles (hors sol).
    fn contact(&self, center: &Vec3, radius: f64) -> Option<Contact>;
}

/// Monde vide (sol seul).
#[derive(Debug, Clone, Copy, Default)]
pub struct EmptyWorld;

impl Collider for EmptyWorld {
    fn raycast(&self, o: &Vec3, d: &Vec3, max: f64) -> Option<RayHit> {
        ray_ground(o, d, max)
    }
    fn contact(&self, _: &Vec3, _: f64) -> Option<Contact> {
        None
    }
}

/// Monde de capacité fixe `N` (aucune allocation).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct World<const N: usize> {
    shapes: [Shape; N],
    len: usize,
}

impl<const N: usize> Default for World<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> World<N> {
    /// Monde vide.
    pub const fn new() -> Self {
        Self { shapes: [Shape::Sphere { center: [0.0; 3], radius: 0.0 }; N], len: 0 }
    }

    /// Ajoute un obstacle. Le rend si le monde est plein.
    pub fn push(&mut self, s: Shape) -> Result<(), Shape> {
        if self.len == N {
            return Err(s);
        }
        self.shapes[self.len] = s;
        self.len += 1;
        Ok(())
    }

    /// Retire tous les obstacles.
    pub fn clear(&mut self) {
        self.len = 0;
    }

    /// Les obstacles.
    pub fn shapes(&self) -> &[Shape] {
        &self.shapes[..self.len]
    }
}

impl<const N: usize> Collider for World<N> {
    fn raycast(&self, o: &Vec3, d: &Vec3, max: f64) -> Option<RayHit> {
        let mut best = ray_ground(o, d, max);
        for (i, s) in self.shapes().iter().enumerate() {
            let limit = best.map(|h| h.distance).unwrap_or(max);
            if let Some((t, n)) = ray_shape(s, o, d, limit) {
                best = Some(RayHit {
                    distance: t,
                    point: add(o, &scale(d, t)),
                    normal: n,
                    obstacle: Some(i),
                });
            }
        }
        best
    }

    fn contact(&self, c: &Vec3, r: f64) -> Option<Contact> {
        let mut best: Option<Contact> = None;
        for s in self.shapes() {
            if let Some(k) = sphere_contact(s, c, r) {
                if best.map(|b| k.depth > b.depth).unwrap_or(true) {
                    best = Some(k);
                }
            }
        }
        best
    }
}

// ---------------------------------------------------------------------------
// Lancer de rayons
// ---------------------------------------------------------------------------

fn ray_ground(o: &Vec3, d: &Vec3, max: f64) -> Option<RayHit> {
    if o[1] > 0.0 && d[1] < -1e-12 {
        let t = -o[1] / d[1];
        if t <= max {
            return Some(RayHit {
                distance: t,
                point: add(o, &scale(d, t)),
                normal: [0.0, 1.0, 0.0],
                obstacle: None,
            });
        }
    }
    None
}

fn ray_shape(s: &Shape, o: &Vec3, d: &Vec3, max: f64) -> Option<(f64, Vec3)> {
    let (t, n) = match *s {
        Shape::Sphere { center, radius } => ray_sphere(&center, radius, o, d)?,
        Shape::Box { center, half, yaw } => ray_box(&center, &half, yaw, o, d)?,
        Shape::Cylinder { base, radius, height } => ray_cylinder(&base, radius, height, o, d)?,
    };
    if t >= 0.0 && t <= max {
        Some((t, n))
    } else {
        None
    }
}

fn ray_sphere(c: &Vec3, r: f64, o: &Vec3, d: &Vec3) -> Option<(f64, Vec3)> {
    let oc = sub(o, c);
    let b = dot(&oc, d);
    let cc = dot(&oc, &oc) - r * r;
    if cc < 0.0 {
        return None; // origine à l'intérieur
    }
    let disc = b * b - cc;
    if disc < 0.0 {
        return None;
    }
    let t = -b - sqrt(disc);
    if t < 0.0 {
        return None;
    }
    let p = add(o, &scale(d, t));
    let n = scale(&sub(&p, c), 1.0 / r);
    Some((t, n))
}

#[inline]
fn to_local(yaw: f64, v: &Vec3) -> Vec3 {
    let (s, c) = (sin(yaw), cos(yaw));
    [c * v[0] - s * v[2], v[1], s * v[0] + c * v[2]]
}

#[inline]
fn to_world(yaw: f64, v: &Vec3) -> Vec3 {
    let (s, c) = (sin(yaw), cos(yaw));
    [c * v[0] + s * v[2], v[1], -s * v[0] + c * v[2]]
}

fn ray_box(center: &Vec3, half: &Vec3, yaw: f64, o: &Vec3, d: &Vec3) -> Option<(f64, Vec3)> {
    let p = to_local(yaw, &sub(o, center));
    let dl = to_local(yaw, d);
    let mut tmin = f64::NEG_INFINITY;
    let mut tmax = f64::INFINITY;
    let mut normal = [0.0; 3];
    for i in 0..3 {
        if fabs(dl[i]) < 1e-12 {
            if fabs(p[i]) > half[i] {
                return None;
            }
        } else {
            let mut t1 = (-half[i] - p[i]) / dl[i];
            let mut t2 = (half[i] - p[i]) / dl[i];
            if t1 > t2 {
                core::mem::swap(&mut t1, &mut t2);
            }
            if t1 > tmin {
                tmin = t1;
                normal = [0.0; 3];
                normal[i] = if dl[i] > 0.0 { -1.0 } else { 1.0 };
            }
            if t2 < tmax {
                tmax = t2;
            }
            if tmin > tmax {
                return None;
            }
        }
    }
    if tmin < 0.0 {
        return None; // origine à l'intérieur ou derrière
    }
    Some((tmin, to_world(yaw, &normal)))
}

fn ray_cylinder(base: &Vec3, r: f64, h: f64, o: &Vec3, d: &Vec3) -> Option<(f64, Vec3)> {
    let op = sub(o, base);
    let a = d[0] * d[0] + d[2] * d[2];
    let b = op[0] * d[0] + op[2] * d[2];
    let c = op[0] * op[0] + op[2] * op[2] - r * r;
    if c < 0.0 && op[1] >= 0.0 && op[1] <= h {
        return None; // origine à l'intérieur
    }
    let mut best: Option<(f64, Vec3)> = None;
    let mut consider = |t: f64, n: Vec3| {
        if t >= 0.0 && best.map(|(bt, _)| t < bt).unwrap_or(true) {
            best = Some((t, n));
        }
    };
    if a > 1e-12 {
        let disc = b * b - a * c;
        if disc >= 0.0 {
            let t = (-b - sqrt(disc)) / a;
            let y = op[1] + t * d[1];
            if y >= 0.0 && y <= h {
                let px = op[0] + t * d[0];
                let pz = op[2] + t * d[2];
                consider(t, [px / r, 0.0, pz / r]);
            }
        }
    }
    if fabs(d[1]) > 1e-12 {
        for (plane, n) in [(0.0, [0.0, -1.0, 0.0]), (h, [0.0, 1.0, 0.0])] {
            let t = (plane - op[1]) / d[1];
            let px = op[0] + t * d[0];
            let pz = op[2] + t * d[2];
            if px * px + pz * pz <= r * r {
                consider(t, n);
            }
        }
    }
    best
}

// ---------------------------------------------------------------------------
// Contacts sphère / forme
// ---------------------------------------------------------------------------

fn sphere_contact(s: &Shape, c: &Vec3, r: f64) -> Option<Contact> {
    match *s {
        Shape::Sphere { center, radius } => {
            let dv = sub(c, &center);
            let dist = norm(&dv);
            let depth = r + radius - dist;
            if depth <= 0.0 {
                return None;
            }
            let normal = if dist > 1e-9 { scale(&dv, 1.0 / dist) } else { [0.0, 1.0, 0.0] };
            Some(Contact { normal, depth })
        }
        Shape::Box { center, half, yaw } => {
            let p = to_local(yaw, &sub(c, &center));
            let mut closest = [0.0; 3];
            for i in 0..3 {
                closest[i] = p[i].clamp(-half[i], half[i]);
            }
            let delta = sub(&p, &closest);
            let d = norm(&delta);
            if d > 1e-9 {
                if d >= r {
                    return None;
                }
                return Some(Contact {
                    normal: to_world(yaw, &scale(&delta, 1.0 / d)),
                    depth: r - d,
                });
            }
            // Centre à l'intérieur : sortie par la face la plus proche.
            let mut axis = 0;
            let mut pen = half[0] - fabs(p[0]);
            for i in 1..3 {
                let q = half[i] - fabs(p[i]);
                if q < pen {
                    pen = q;
                    axis = i;
                }
            }
            let mut n = [0.0; 3];
            n[axis] = if p[axis] >= 0.0 { 1.0 } else { -1.0 };
            Some(Contact { normal: to_world(yaw, &n), depth: pen + r })
        }
        Shape::Cylinder { base, radius, height } => {
            let p = sub(c, &base);
            let radial = sqrt(p[0] * p[0] + p[2] * p[2]);
            let k = if radial > radius { radius / radial } else { 1.0 };
            let closest = [p[0] * k, p[1].clamp(0.0, height), p[2] * k];
            let delta = sub(&p, &closest);
            let d = norm(&delta);
            if d > 1e-9 {
                if d >= r {
                    return None;
                }
                return Some(Contact { normal: scale(&delta, 1.0 / d), depth: r - d });
            }
            let side = radius - radial;
            let down = p[1];
            let up = height - p[1];
            if side <= down && side <= up {
                let n = if radial > 1e-9 { [p[0] / radial, 0.0, p[2] / radial] } else { [1.0, 0.0, 0.0] };
                Some(Contact { normal: n, depth: side + r })
            } else if down < up {
                Some(Contact { normal: [0.0, -1.0, 0.0], depth: down + r })
            } else {
                Some(Contact { normal: [0.0, 1.0, 0.0], depth: up + r })
            }
        }
    }
}
