//! Rotations 3D : le groupe SO(3).

use core::f64::consts::PI;
use core::ops::Mul;

use crate::linalg::{
    cofactor, det, dot, mat_add, mat_mul, mat_scale, mat_vec, norm, scale, transpose, Mat3,
    Vec3, MAT3_IDENTITY,
};

/// Seuil (sur θ²) sous lequel on bascule sur les développements de Taylor.
const TAYLOR_EPS2: f64 = 1e-4;

/// Rotation 3D, élément de SO(3), stockée sous forme de matrice 3×3 (par lignes).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rotation3 {
    /// Matrice de rotation, `matrix[i][j]` = ligne `i`, colonne `j`.
    pub matrix: Mat3,
}

impl Default for Rotation3 {
    #[inline]
    fn default() -> Self {
        Self::identity()
    }
}

impl Rotation3 {
    // ------------------------------------------------------------------
    // Construction
    // ------------------------------------------------------------------

    /// Rotation identité.
    #[inline]
    pub const fn identity() -> Self {
        Self {
            matrix: MAT3_IDENTITY,
        }
    }

    /// Exponentielle so(3) → SO(3) (formule de Rodrigues).
    ///
    /// Stable pour θ → 0 grâce aux développements de Taylor.
    pub fn exp(omega: &Vec3) -> Self {
        let th2 = dot(omega, omega);
        let (a, b) = if th2 < TAYLOR_EPS2 {
            (
                1.0 - th2 / 6.0 * (1.0 - th2 / 20.0),
                0.5 - th2 / 24.0 * (1.0 - th2 / 30.0),
            )
        } else {
            let th = libm::sqrt(th2);
            let h = libm::sin(0.5 * th);
            (libm::sin(th) / th, 2.0 * h * h / th2)
        };
        let k = Self::hat(omega);
        let k2 = mat_mul(&k, &k);
        let m = mat_add(&mat_add(&MAT3_IDENTITY, &mat_scale(&k, a)), &mat_scale(&k2, b));
        Self { matrix: m }
    }

    /// Rotation d'angle `angle` (rad) autour de `axis` (normalisé en interne).
    /// Un axe nul donne l'identité.
    pub fn from_axis_angle(axis: &Vec3, angle: f64) -> Self {
        let n = norm(axis);
        if n == 0.0 {
            return Self::identity();
        }
        Self::exp(&scale(axis, angle / n))
    }

    /// Depuis un quaternion `[x, y, z, w]` (normalisé en interne).
    /// Un quaternion nul donne l'identité.
    pub fn from_quaternion(q: &[f64; 4]) -> Self {
        let n = libm::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if n == 0.0 {
            return Self::identity();
        }
        let (x, y, z, w) = (q[0] / n, q[1] / n, q[2] / n, q[3] / n);
        Self {
            matrix: [
                [
                    1.0 - 2.0 * (y * y + z * z),
                    2.0 * (x * y - z * w),
                    2.0 * (x * z + y * w),
                ],
                [
                    2.0 * (x * y + z * w),
                    1.0 - 2.0 * (x * x + z * z),
                    2.0 * (y * z - x * w),
                ],
                [
                    2.0 * (x * z - y * w),
                    2.0 * (y * z + x * w),
                    1.0 - 2.0 * (x * x + y * y),
                ],
            ],
        }
    }

    // ------------------------------------------------------------------
    // Algèbre de Lie
    // ------------------------------------------------------------------

    /// Opérateur `^` : vecteur → matrice antisymétrique.
    #[inline]
    pub fn hat(omega: &Vec3) -> Mat3 {
        [
            [0.0, -omega[2], omega[1]],
            [omega[2], 0.0, -omega[0]],
            [-omega[1], omega[0], 0.0],
        ]
    }

    /// Opérateur `∨` : inverse de [`Rotation3::hat`].
    #[inline]
    pub fn vee(m: &Mat3) -> Vec3 {
        [m[2][1], m[0][2], m[1][0]]
    }

    /// Partie antisymétrique (sin θ · axe) et cos θ de la matrice.
    #[inline]
    fn sin_vec_and_cos(&self) -> (Vec3, f64) {
        let m = &self.matrix;
        let sv = [
            0.5 * (m[2][1] - m[1][2]),
            0.5 * (m[0][2] - m[2][0]),
            0.5 * (m[1][0] - m[0][1]),
        ];
        let c = 0.5 * (m[0][0] + m[1][1] + m[2][2] - 1.0);
        (sv, c)
    }

    /// Logarithme SO(3) → so(3), de norme ≤ π.
    ///
    /// Robuste près de l'identité (série) et près de θ = π (extraction de
    /// l'axe via la partie symétrique).
    pub fn log(&self) -> Vec3 {
        let (sv, c) = self.sin_vec_and_cos();
        let s = norm(&sv);
        let th = libm::atan2(s, c);
        let th2 = th * th;

        if th2 < TAYLOR_EPS2 {
            // θ / sin θ = 1 + θ²/6 + 7θ⁴/360 + ...
            return scale(&sv, 1.0 + th2 / 6.0 * (1.0 + 7.0 * th2 / 60.0));
        }

        if th > PI - 1e-2 {
            // Près de π, sin θ → 0 : l'axe se lit dans la partie symétrique
            // S = cos θ I + (1 - cos θ) a aᵀ.
            let m = &self.matrix;
            let k = 1.0 - c;
            let mut i = 0;
            if m[1][1] > m[i][i] {
                i = 1;
            }
            if m[2][2] > m[i][i] {
                i = 2;
            }
            let mut col = [0.0; 3];
            for (j, cj) in col.iter_mut().enumerate() {
                let sym = 0.5 * (m[j][i] + m[i][j]);
                *cj = (if j == i { sym - c } else { sym }) / k;
            }
            let n = norm(&col);
            if n == 0.0 {
                return [0.0; 3];
            }
            let mut a = scale(&col, 1.0 / n);
            if dot(&a, &sv) < 0.0 {
                a = scale(&a, -1.0);
            }
            return scale(&a, th);
        }

        scale(&sv, th / s)
    }

    /// Angle de rotation dans [0, π].
    #[inline]
    pub fn angle(&self) -> f64 {
        let (sv, c) = self.sin_vec_and_cos();
        libm::atan2(norm(&sv), c)
    }

    /// Jacobienne gauche `J(ω) = I + b K + c K²`.
    pub fn left_jacobian(omega: &Vec3) -> Mat3 {
        let th2 = dot(omega, omega);
        let (b, c) = if th2 < TAYLOR_EPS2 {
            (
                0.5 - th2 / 24.0 * (1.0 - th2 / 30.0),
                1.0 / 6.0 - th2 / 120.0 * (1.0 - th2 / 42.0),
            )
        } else {
            let th = libm::sqrt(th2);
            let h = libm::sin(0.5 * th);
            (
                2.0 * h * h / th2,
                (th - libm::sin(th)) / (th2 * th),
            )
        };
        let k = Self::hat(omega);
        let k2 = mat_mul(&k, &k);
        mat_add(&mat_add(&MAT3_IDENTITY, &mat_scale(&k, b)), &mat_scale(&k2, c))
    }

    /// Inverse de la jacobienne gauche, `J(ω)⁻¹ = I - K/2 + d K²`.
    /// Valable pour `‖ω‖ < 2π`.
    pub fn left_jacobian_inv(omega: &Vec3) -> Mat3 {
        let th2 = dot(omega, omega);
        let d = if th2 < TAYLOR_EPS2 {
            1.0 / 12.0 + th2 / 720.0 * (1.0 + th2 / 42.0)
        } else {
            let th = libm::sqrt(th2);
            let h = libm::sin(0.5 * th);
            (1.0 - th * libm::sin(th) / (4.0 * h * h)) / th2
        };
        let k = Self::hat(omega);
        let k2 = mat_mul(&k, &k);
        mat_add(
            &mat_add(&MAT3_IDENTITY, &mat_scale(&k, -0.5)),
            &mat_scale(&k2, d),
        )
    }

    // ------------------------------------------------------------------
    // Opérations de groupe
    // ------------------------------------------------------------------

    /// Composition `self ∘ other` (applique `other` puis `self`).
    #[inline]
    pub fn compose(&self, other: &Self) -> Self {
        Self {
            matrix: mat_mul(&self.matrix, &other.matrix),
        }
    }

    /// Inverse (transposée).
    #[inline]
    pub fn inverse(&self) -> Self {
        Self {
            matrix: transpose(&self.matrix),
        }
    }

    /// Applique la rotation à un vecteur.
    #[inline]
    pub fn rotate(&self, v: &Vec3) -> Vec3 {
        mat_vec(&self.matrix, v)
    }

    /// Angle de la rotation relative, dans [0, π].
    #[inline]
    pub fn geodesic_distance(&self, other: &Self) -> f64 {
        self.inverse().compose(other).angle()
    }

    /// Interpolation géodésique : `t = 0` → `self`, `t = 1` → `other`.
    pub fn interpolate(&self, other: &Self, t: f64) -> Self {
        let rel = self.inverse().compose(other).log();
        self.compose(&Self::exp(&scale(&rel, t)))
    }

    // ------------------------------------------------------------------
    // Intégration
    // ------------------------------------------------------------------

    /// `R ← R · exp(ω dt)`, ω dans le repère **corps**. Exacte pour ω constant.
    #[inline]
    pub fn integrate(&self, omega_body: &Vec3, dt: f64) -> Self {
        self.compose(&Self::exp(&scale(omega_body, dt)))
    }

    /// `R ← exp(ω dt) · R`, ω dans le repère **monde**.
    #[inline]
    pub fn integrate_world(&self, omega_world: &Vec3, dt: f64) -> Self {
        Self::exp(&scale(omega_world, dt)).compose(self)
    }

    /// Intègre un lot de rotations **en place**, sans allocation.
    ///
    /// # Panics
    /// Si `rotations` et `omegas` n'ont pas la même longueur.
    pub fn integrate_batch_in_place(rotations: &mut [Self], omegas: &[Vec3], dt: f64) {
        assert_eq!(
            rotations.len(),
            omegas.len(),
            "rotations et omegas doivent avoir la même longueur"
        );
        for (r, w) in rotations.iter_mut().zip(omegas.iter()) {
            *r = r.integrate(w, dt);
        }
    }

    /// Intègre un lot de rotations et retourne un nouveau `Vec`.
    /// Disponible avec la feature `alloc`.
    ///
    /// # Panics
    /// Si `rotations` et `omegas` n'ont pas la même longueur.
    #[cfg(feature = "alloc")]
    pub fn integrate_batch(
        rotations: &[Self],
        omegas: &[Vec3],
        dt: f64,
    ) -> alloc::vec::Vec<Self> {
        assert_eq!(
            rotations.len(),
            omegas.len(),
            "rotations et omegas doivent avoir la même longueur"
        );
        rotations
            .iter()
            .zip(omegas.iter())
            .map(|(r, w)| r.integrate(w, dt))
            .collect()
    }

    // ------------------------------------------------------------------
    // Contrôle de la dérive numérique
    // ------------------------------------------------------------------

    /// Erreur d'orthogonalité `‖RᵀR − I‖_F`.
    pub fn orthogonality_error(&self) -> f64 {
        let rtr = mat_mul(&transpose(&self.matrix), &self.matrix);
        let mut acc = 0.0;
        for i in 0..3 {
            for j in 0..3 {
                let d = rtr[i][j] - MAT3_IDENTITY[i][j];
                acc += d * d;
            }
        }
        libm::sqrt(acc)
    }

    /// Vrai si `R` est orthogonale (à `tol` près) de déterminant +1.
    /// Toujours faux en présence de `NaN`.
    pub fn is_valid(&self, tol: f64) -> bool {
        self.orthogonality_error() <= tol && libm::fabs(det(&self.matrix) - 1.0) <= tol
    }

    /// Reprojette sur SO(3) par itération polaire
    /// (`R ← (R + R⁻ᵀ) / 2`, convergence quadratique).
    pub fn renormalized(&self) -> Self {
        let mut r = self.matrix;
        for _ in 0..8 {
            let d = det(&r);
            if d == 0.0 {
                return Self::identity();
            }
            let inv_t = mat_scale(&cofactor(&r), 1.0 / d);
            let next = mat_scale(&mat_add(&r, &inv_t), 0.5);
            let mut diff = 0.0;
            for i in 0..3 {
                for j in 0..3 {
                    diff += libm::fabs(next[i][j] - r[i][j]);
                }
            }
            r = next;
            if diff < 1e-15 {
                break;
            }
        }
        Self { matrix: r }
    }

    // ------------------------------------------------------------------
    // Conversions
    // ------------------------------------------------------------------

    /// Quaternion unitaire `[x, y, z, w]` avec `w ≥ 0` (méthode de Shepperd).
    pub fn to_quaternion(&self) -> [f64; 4] {
        let m = &self.matrix;
        let tr = m[0][0] + m[1][1] + m[2][2];
        let (x, y, z, w);
        if tr > 0.0 {
            let s = 2.0 * libm::sqrt(tr + 1.0);
            w = 0.25 * s;
            x = (m[2][1] - m[1][2]) / s;
            y = (m[0][2] - m[2][0]) / s;
            z = (m[1][0] - m[0][1]) / s;
        } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
            let s = 2.0 * libm::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]);
            w = (m[2][1] - m[1][2]) / s;
            x = 0.25 * s;
            y = (m[0][1] + m[1][0]) / s;
            z = (m[0][2] + m[2][0]) / s;
        } else if m[1][1] > m[2][2] {
            let s = 2.0 * libm::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]);
            w = (m[0][2] - m[2][0]) / s;
            x = (m[0][1] + m[1][0]) / s;
            y = 0.25 * s;
            z = (m[1][2] + m[2][1]) / s;
        } else {
            let s = 2.0 * libm::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]);
            w = (m[1][0] - m[0][1]) / s;
            x = (m[0][2] + m[2][0]) / s;
            y = (m[1][2] + m[2][1]) / s;
            z = 0.25 * s;
        }
        let n = libm::sqrt(x * x + y * y + z * z + w * w);
        let sign = if w < 0.0 { -1.0 } else { 1.0 };
        let k = sign / n;
        [x * k, y * k, z * k, w * k]
    }
}

impl Mul<Rotation3> for Rotation3 {
    type Output = Rotation3;
    #[inline]
    fn mul(self, rhs: Rotation3) -> Rotation3 {
        self.compose(&rhs)
    }
}

impl Mul<Vec3> for Rotation3 {
    type Output = Vec3;
    #[inline]
    fn mul(self, rhs: Vec3) -> Vec3 {
        self.rotate(&rhs)
    }
}
