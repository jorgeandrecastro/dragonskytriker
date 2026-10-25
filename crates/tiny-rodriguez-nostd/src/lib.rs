//! # tiny-rodriguez-nostd
//!
//! Bibliothèque minimale de **géométrie différentielle** pour la robotique et
//! les drones, version `#![no_std]` de `tiny-rodriguez-std` :
//!
//! - **SO(3)** : rotations 3D ([`Rotation3`])
//! - **SE(3)** : poses rigides, rotation + translation ([`Pose3`])
//! - **Intégration exponentielle** exacte pour une vitesse constante
//!
//! Aucune allocation, aucun `unsafe`, une seule dépendance (`libm`).
//! Compatible microcontrôleurs (Cortex-M, RISC-V, ESP32...), WASM et `std`.
//!
//! La feature optionnelle `alloc` ajoute les fonctions `integrate_batch`
//! qui retournent un `Vec`.
//!
//! ## Conventions
//!
//! - Matrices stockées par **lignes** : `m[i][j]` = ligne `i`, colonne `j`.
//! - Quaternions `[x, y, z, w]`, Hamilton, `w >= 0` en sortie.
//! - Torseurs `[vx, vy, vz, wx, wy, wz]` : linéaire d'abord, puis angulaire.
//! - `integrate` : vitesse dans le repère **corps** (`R <- R * exp(w dt)`).
//! - `integrate_world` : vitesse dans le repère **monde** (`R <- exp(w dt) * R`).

#![no_std]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

#[cfg(feature = "alloc")]
extern crate alloc;

pub mod linalg;
mod se3;
mod so3;

pub use linalg::{Mat3, Vec3, MAT3_IDENTITY};
pub use se3::Pose3;
pub use so3::Rotation3;
