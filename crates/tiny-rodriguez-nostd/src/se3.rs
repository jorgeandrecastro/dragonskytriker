//! Poses rigides 3D : le groupe SE(3).

use crate::linalg::{add, mat_vec, scale, Vec3};
use crate::so3::Rotation3;

/// Pose rigide 3D (rotation + translation), élément de SE(3).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose3 {
    /// Partie rotation.
    pub rotation: Rotation3,
    /// Partie translation.
    pub translation: Vec3,
}

impl Default for Pose3 {
    #[inline]
    fn default() -> Self {
        Self::identity()
    }
}

impl Pose3 {
    /// Pose identité.
    #[inline]
    pub const fn identity() -> Self {
        Self {
            rotation: Rotation3::identity(),
            translation: [0.0; 3],
        }
    }

    /// Construit une pose.
    #[inline]
    pub const fn new(rotation: Rotation3, translation: Vec3) -> Self {
        Self {
            rotation,
            translation,
        }
    }

    /// Exponentielle se(3) → SE(3) : `R = exp(ω)`, `t = J(ω)·v`.
    /// Torseur `ξ = [vx, vy, vz, ωx, ωy, ωz]`.
    pub fn exp(xi: &[f64; 6]) -> Self {
        let v = [xi[0], xi[1], xi[2]];
        let w = [xi[3], xi[4], xi[5]];
        Self {
            rotation: Rotation3::exp(&w),
            translation: mat_vec(&Rotation3::left_jacobian(&w), &v),
        }
    }

    /// Logarithme SE(3) → se(3) : `ω = log R`, `v = J(ω)⁻¹·t`.
    pub fn log(&self) -> [f64; 6] {
        let w = self.rotation.log();
        let v = mat_vec(&Rotation3::left_jacobian_inv(&w), &self.translation);
        [v[0], v[1], v[2], w[0], w[1], w[2]]
    }

    /// Composition `self ∘ other` (applique `other` puis `self`).
    #[inline]
    pub fn compose(&self, other: &Self) -> Self {
        Self {
            rotation: self.rotation.compose(&other.rotation),
            translation: add(&self.rotation.rotate(&other.translation), &self.translation),
        }
    }

    /// Inverse `(Rᵀ, −Rᵀt)`.
    #[inline]
    pub fn inverse(&self) -> Self {
        let rt = self.rotation.inverse();
        Self {
            translation: scale(&rt.rotate(&self.translation), -1.0),
            rotation: rt,
        }
    }

    /// Transforme un point : `R p + t`.
    #[inline]
    pub fn transform_point(&self, p: &Vec3) -> Vec3 {
        add(&self.rotation.rotate(p), &self.translation)
    }

    /// `T ← T · exp(ξ dt)`, torseur dans le repère **corps**.
    /// Exacte pour ξ constant.
    #[inline]
    pub fn integrate(&self, twist_body: &[f64; 6], dt: f64) -> Self {
        let xi = [
            twist_body[0] * dt,
            twist_body[1] * dt,
            twist_body[2] * dt,
            twist_body[3] * dt,
            twist_body[4] * dt,
            twist_body[5] * dt,
        ];
        self.compose(&Self::exp(&xi))
    }

    /// Intègre un lot de poses **en place**, sans allocation.
    ///
    /// # Panics
    /// Si `poses` et `twists` n'ont pas la même longueur.
    pub fn integrate_batch_in_place(poses: &mut [Self], twists: &[[f64; 6]], dt: f64) {
        assert_eq!(
            poses.len(),
            twists.len(),
            "poses et twists doivent avoir la même longueur"
        );
        for (p, t) in poses.iter_mut().zip(twists.iter()) {
            *p = p.integrate(t, dt);
        }
    }

    /// Intègre un lot de poses et retourne un nouveau `Vec`.
    /// Disponible avec la feature `alloc`.
    ///
    /// # Panics
    /// Si `poses` et `twists` n'ont pas la même longueur.
    #[cfg(feature = "alloc")]
    pub fn integrate_batch(
        poses: &[Self],
        twists: &[[f64; 6]],
        dt: f64,
    ) -> alloc::vec::Vec<Self> {
        assert_eq!(
            poses.len(),
            twists.len(),
            "poses et twists doivent avoir la même longueur"
        );
        poses
            .iter()
            .zip(twists.iter())
            .map(|(p, t)| p.integrate(t, dt))
            .collect()
    }
}
