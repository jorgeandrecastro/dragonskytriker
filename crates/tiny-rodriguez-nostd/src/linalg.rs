//! Algèbre linéaire 3×3 minimale, sans allocation.

/// Vecteur 3D.
pub type Vec3 = [f64; 3];
/// Matrice 3×3 stockée par lignes : `m[i][j]` = ligne `i`, colonne `j`.
pub type Mat3 = [[f64; 3]; 3];

/// Matrice identité 3×3.
pub const MAT3_IDENTITY: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// Produit scalaire.
#[inline]
pub fn dot(a: &Vec3, b: &Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Produit vectoriel `a × b`.
#[inline]
pub fn cross(a: &Vec3, b: &Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Norme euclidienne.
#[inline]
pub fn norm(a: &Vec3) -> f64 {
    libm::sqrt(dot(a, a))
}

/// Multiplication d'un vecteur par un scalaire.
#[inline]
pub fn scale(a: &Vec3, s: f64) -> Vec3 {
    [a[0] * s, a[1] * s, a[2] * s]
}

/// Somme de deux vecteurs.
#[inline]
pub fn add(a: &Vec3, b: &Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

/// Différence de deux vecteurs.
#[inline]
pub fn sub(a: &Vec3, b: &Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// Produit matrice × matrice.
#[inline]
pub fn mat_mul(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut r = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            r[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
    r
}

/// Produit matrice × vecteur.
#[inline]
pub fn mat_vec(a: &Mat3, v: &Vec3) -> Vec3 {
    [dot(&a[0], v), dot(&a[1], v), dot(&a[2], v)]
}

/// Transposée.
#[inline]
pub fn transpose(a: &Mat3) -> Mat3 {
    [
        [a[0][0], a[1][0], a[2][0]],
        [a[0][1], a[1][1], a[2][1]],
        [a[0][2], a[1][2], a[2][2]],
    ]
}

/// Déterminant.
#[inline]
pub fn det(a: &Mat3) -> f64 {
    a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
}

/// Somme de deux matrices.
#[inline]
pub fn mat_add(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut r = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            r[i][j] = a[i][j] + b[i][j];
        }
    }
    r
}

/// Multiplication d'une matrice par un scalaire.
#[inline]
pub fn mat_scale(a: &Mat3, s: f64) -> Mat3 {
    let mut r = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            r[i][j] = a[i][j] * s;
        }
    }
    r
}

/// Matrice des cofacteurs. Pour `A` inversible : `A⁻ᵀ = cof(A) / det(A)`.
#[inline]
pub(crate) fn cofactor(a: &Mat3) -> Mat3 {
    [
        [
            a[1][1] * a[2][2] - a[1][2] * a[2][1],
            a[1][2] * a[2][0] - a[1][0] * a[2][2],
            a[1][0] * a[2][1] - a[1][1] * a[2][0],
        ],
        [
            a[0][2] * a[2][1] - a[0][1] * a[2][2],
            a[0][0] * a[2][2] - a[0][2] * a[2][0],
            a[0][1] * a[2][0] - a[0][0] * a[2][1],
        ],
        [
            a[0][1] * a[1][2] - a[0][2] * a[1][1],
            a[0][2] * a[1][0] - a[0][0] * a[1][2],
            a[0][0] * a[1][1] - a[0][1] * a[1][0],
        ],
    ]
}
