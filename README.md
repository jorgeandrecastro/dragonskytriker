# DragonSkyTriker

Simulateur de drones basé sur **Bevy 0.20** : physique, capteurs réalistes, estimation d'état, mondes en JSON, et un rendu épuré.

![Studio](docs/studio.png)

## Ce que ça fait

| | |
|---|---|
| **Physique** | Corps rigide intégré sur SE(3) (`tiny-rodriguez-nostd`), moteurs du 1er ordre, traînée linéaire + quadratique par rapport à l'air, **vent et rafales** (Ornstein-Uhlenbeck), densité d'air, **effet de sol**, **batterie LiPo 4S** (tension en charge, poussée ∝ V², puissance par théorie du disque), **collisions** (sphère, boîte, cylindre) avec restitution, destruction à l'impact. |
| **Contrôle** | Contrôleur géométrique en cascade sur SO(3) : position en racine carrée (freinage sans dépassement), vitesse avec **intégrateur anti-windup** (vent, erreur de masse), compensation de la traînée linéaire + quadratique, attitude par `log(Rᵀ R_d)`. |
| **Capteurs** | **IMU** (bruit blanc + marche aléatoire des biais), **baromètre** (atmosphère standard, dérive), **magnétomètre** (fer dur), **GNSS** (10 Hz, erreur corrélée, latence 0,12 s, lat/lon), **télémètre**, **lidar 360° × 4 canaux** par lancer de rayons. Chacun a sa cadence. |
| **Estimation** | Filtre de **Mahony sur SO(3)** (gyro + gravité + cap magnétique, biais gyro estimé) et position/vitesse par filtre complémentaire GNSS + baromètre, **latence compensée**. Le drone peut **voler sur ses capteurs** (`E`) au lieu de la vérité. |
| **Mondes** | Fichiers JSON : obstacles, vent, mission. `DST_WORLD=assets/worlds/slalom.json cargo run --release` |
| **Headless** | Même cœur sans fenêtre : **≈ ×1000 le temps réel**, déterministe (graine), export CSV. |
| **Rendu** | Studio blanc infini, ombres douces, HUD en verre dépoli (Inter), retours lidar, vent, fantôme de l'estimation. |

## Architecture

| Crate | Rôle | `no_std` |
|---|---|---|
| `tiny-rodriguez-nostd` | Géométrie SO(3)/SE(3) | oui |
| `dragonskytriker-core` | Dynamique, environnement, monde, capteurs, estimateur, contrôle, autopilote, `Simulation` | oui |
| `dragonskytriker` | Plugin Bevy, mondes JSON, HUD, enregistreur | non |

Le cœur ne dépend pas de Bevy : le même code peut tourner dans un firmware, en CI, ou en lot sur un serveur.

**Repère** : Bevy (Y haut, avant −Z, est +X, nord −Z). Unités SI.

## Lancer

```
cargo run --release                       # fenêtre
cargo run --release -p dragonskytriker-core --example headless -- 300 log.csv 3
```

Rust ≥ 1.95 (Bevy 0.19). Linux : `libasound2-dev libudev-dev libwayland-dev libxkbcommon-dev pkg-config`.

## Touches

Les touches-lettres sont **logiques** : elles suivent la lettre imprimée sur la touche, quelle que soit la disposition (AZERTY, QWERTY…), et ne dépendent pas des majuscules.

| Touches | Action |
|---|---|
| `T` `L` `H` `M` | Décollage, atterrissage, maintien, mission |
| `W` `A` `S` `D` ou `Z` `Q` `S` `D` | Pilotage manuel : avant, gauche, arrière, droite |
| `Espace` ou `↑` / `↓` | Monter / descendre |
| `←` `→` | Cap (lacet) |
| `E` | Voler sur l'estimation (capteurs) plutôt que sur la vérité |
| `G` | Vent : calme → brise → rafales → tempête |
| `V` | Afficher/masquer lidar, vent, estimation |
| `I` / `Tab` | Carte capteurs / toute l'interface |
| `F` | Enregistrer `flight_log.csv` |
| `C` `R` | Caméra (suivi, orbite, panorama) / reset |

## Utiliser le SDK

```rust
use bevy::prelude::*;
use dragonskytriker::{DragonSkyPlugins, Flight};
use dragonskytriker::core::Command;

fn main() {
    App::new().add_plugins((DefaultPlugins, DragonSkyPlugins)).add_systems(Update, my_logic).run();
}

fn my_logic(mut flight: ResMut<Flight>) {
    let imu = flight.sensors.latest.imu;            // mesures bruitées
    let nearest = flight.sensors.latest.lidar.nearest();
    let state = flight.control_state();
    if nearest.is_some_and(|d| d < 1.0) {
        flight.autopilot.command(Command::Hover, &state);   // évitement minimal
    }
}
```

Sans fenêtre :

```rust
use dragonskytriker_core::*;
let mut sim = Simulation::new(QuadParams::default(), Gains::default(), 42);
sim.env.wind = Wind::new([3.0, 0.0, 0.0], 1.0, 42);
sim.use_estimator = true;
sim.autopilot.command(Command::Takeoff { altitude: 2.0 }, &sim.quad.state);
for _ in 0..2400 { sim.step(1.0 / 240.0); }
```

## Tests

```
cargo test -p tiny-rodriguez-nostd -p dragonskytriker-core -p dragonskytriker
```

Ils couvrent : atmosphère, RNG, lancer de rayons, contacts, vent, batterie, effet de sol, collisions, statistiques de bruit de l'IMU, baromètre, GNSS, lidar, estimateur (≤ 4,6° d'attitude, mieux que le GNSS seul en position), vol sur capteurs seuls, déterminisme, vol (décollage, goto, lacet, mission, atterrissage, choc violent, dérive de SO(3) sur 120 s), et trois mondes complets (studio, slalom sur vérité, slalom sur capteurs).

## Limites connues

- Pas d'évitement d'obstacles : le drone suit sa mission, les capteurs sont là pour le construire.
- La mission enchaîne les points de survol sans s'arrêter : le drone coupe les virages (jusqu'à ≈ 1,5 m d'un coin). Un point avec `hold` est atteint précisément.
- Le magnétomètre n'est pas calibré en ligne : ≈ 3,5° d'erreur de cap.
- En manœuvre très agressive (inclinaison ≈ 30° sous rafale), l'attitude estimée peut s'écarter de plus de 20° un instant.
- Collisions par sphère englobante (0,17 m) ; pas de dynamique de contact fine.
- Un seul modèle de drone (Dragon X1) : les paramètres (masse, inerties, poussée) sont dans `QuadParams`, pas encore chargeables depuis un fichier.
- Les contrôleurs et le mixeur gardent une copie des paramètres : changer la masse en cours de vol demande de recréer la simulation.

## Feuille de route

- Drone décrit par un fichier JSON (masse, inerties, moteurs, batterie) : `DST_DRONE=assets/drones/mon_drone.json`.
- Réglages de masse et de météo (vent, densité de l'air) depuis l'interface.
- Import d'un modèle 3D (glTF) pour l'apparence, séparé de la physique.
- Contrôleur avec modèle erroné (masse mal connue) pour tester la robustesse.

## Licences

Code : `GPL-2.0-or-later` (copiez votre `LICENSE`). Police Inter : SIL OFL 1.1 (`assets/fonts/OFL.txt`).