# tiny-rodriguez-nostd

Version **`#![no_std]`** de `tiny-rodriguez-std` : bibliothèque minimale de **géométrie différentielle** pour la robotique et les drones.

- **SO(3)** : rotations 3D (`Rotation3`)
- **SE(3)** : poses rigides, rotation + translation (`Pose3`)
- **Intégration exponentielle** exacte pour une vitesse constante

`#![no_std]`, `#![forbid(unsafe_code)]`, sans allocation, une seule dépendance (`libm`). Utilisable sur microcontrôleurs (Cortex-M, RISC-V, ESP32), en WASM, ou dans un moteur comme Bevy. Précision `f64`.

## Installation

```toml
[dependencies]
tiny-rodriguez-nostd = "0.1"

# pour integrate_batch (retourne un Vec, requiert un allocateur global) :
tiny-rodriguez-nostd = { version = "0.1", features = ["alloc"] }
```

Rust minimum : 1.75, édition 2021.

## Démarrage rapide

```rust
use tiny_rodriguez_nostd::{Pose3, Rotation3};

let r = Rotation3::identity().integrate(&[0.5, 1.2, 0.3], 0.016);
let omega = r.log();

let mut pose = Pose3::identity();
for _ in 0..1000 {
    pose = pose.integrate(&[1.0, 0.0, 0.0, 0.0, 0.0, 0.5], 0.001);
}
let xi = pose.log();
```

## Conventions

| Sujet              | Convention                                                         |
|--------------------|--------------------------------------------------------------------|
| Matrices           | Par lignes : `m[i][j]` = ligne `i`, colonne `j`                    |
| Quaternions        | `[x, y, z, w]`, Hamilton, `w ≥ 0` en sortie                        |
| Torseurs           | `[vx, vy, vz, ωx, ωy, ωz]` (linéaire d'abord)                      |
| `integrate`        | Vitesse dans le repère **corps** : `R ← R · exp(ω dt)`             |
| `integrate_world`  | Vitesse dans le repère **monde** : `R ← exp(ω dt) · R`             |
| Composition        | `a.compose(&b)` = appliquer `b` puis `a` (équivaut à `a * b`)      |
| Entrées dégénérées | Axe/quaternion nul → identité. `NaN`/infini : pas de panique       |

## API

Identique à `tiny-rodriguez-std` : `Rotation3` (`identity`, `exp`, `log`, `from_axis_angle`, `from_quaternion`, `to_quaternion`, `hat`, `vee`, `left_jacobian`, `left_jacobian_inv`, `compose`, `inverse`, `rotate`, `geodesic_distance`, `interpolate`, `integrate`, `integrate_world`, `orthogonality_error`, `is_valid`, `renormalized`) et `Pose3` (`identity`, `new`, `exp`, `log`, `compose`, `inverse`, `transform_point`, `integrate`). Le module `linalg` expose l'algèbre 3×3 (`dot`, `cross`, `norm`, `scale`, `add`, `sub`, `mat_mul`, `mat_vec`, `transpose`, `det`, `mat_add`, `mat_scale`).

### Différences avec la version `std`

| Sujet                         | `std`                          | `nostd`                                         |
|-------------------------------|--------------------------------|-------------------------------------------------|
| Fonctions mathématiques       | `f64::sin`, ...                | `libm`                                          |
| `integrate_batch_in_place`    | oui                            | oui (aucune allocation)                         |
| `integrate_batch` (`Vec`)     | toujours                       | feature `alloc`                                 |
| Parallélisme `rayon`          | feature `rayon`                | retiré (pas de threads en `no_std`)             |

## Dérive numérique

Appelez `renormalized()` toutes les quelques centaines de pas sur les intégrations longues et surveillez `orthogonality_error()`.

## Tests

```
cargo test
cargo test --features alloc
cargo run --example flight_loop
```

Les tests sont écrits sans dépendance externe (générateur pseudo-aléatoire interne, 5 000 cas par propriété, angles proches de 0 et de π sur-représentés).

## Utilisation avec Bevy

Cette crate ne dépend pas de Bevy. Pour l'afficher, convertissez à la frontière :

```rust
let q = pose.rotation.to_quaternion();
transform.rotation = bevy::math::Quat::from_xyzw(q[0] as f32, q[1] as f32, q[2] as f32, q[3] as f32);
transform.translation = Vec3::new(t[0] as f32, t[1] as f32, t[2] as f32);
```

## Licence

`GPL-2.0-or-later`, comme `tiny-rodriguez-std`. Copiez le fichier `LICENSE` de ce dépôt à la racine. `libm` est sous MIT.
