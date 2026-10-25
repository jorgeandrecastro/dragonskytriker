# Changelog

## 0.1.0

- Première version `#![no_std]`, portée depuis `tiny-rodriguez-std` 0.2.
- Dépendance unique : `libm` (sin, cos, sqrt, atan2).
- `Rotation3` (SO(3)) et `Pose3` (SE(3)) : mêmes API et conventions que la version `std`.
- Lots : `integrate_batch_in_place` sans allocation ; `integrate_batch` (retourne un `Vec`) derrière la feature `alloc`.
- Retiré par rapport à la version `std` : feature `rayon` (pas de threads en `no_std`).
