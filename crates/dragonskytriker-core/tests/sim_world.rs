use dragonskytriker_core::geometry::linalg::{norm, sub};
use dragonskytriker_core::*;

const DT: f64 = 1.0 / 240.0;

fn run(sim: &mut Simulation, seconds: f64) {
    for _ in 0..(seconds / DT) as usize {
        sim.step(DT);
    }
}

fn airborne(alt: f64) -> Simulation {
    let mut sim = Simulation::default();
    sim.autopilot.command(Command::Takeoff { altitude: alt }, &sim.quad.state);
    run(&mut sim, 7.0);
    sim
}

#[test]
fn atmosphere_matches_standard_values() {
    let a = Atmosphere::default();
    assert!((a.pressure_at(0.0) - 101_325.0).abs() < 1e-6);
    assert!((a.density_at(0.0) - 1.225).abs() < 0.002);
    assert!((a.pressure_at(1000.0) - 89_875.0).abs() < 60.0);
    assert!((a.altitude_from_pressure(a.pressure_at(850.0)) - 850.0).abs() < 1e-6);
}

#[test]
fn rng_is_deterministic_and_gaussian() {
    let (mut a, mut b) = (Rng::new(5), Rng::new(5));
    assert_eq!(a.next_u64(), b.next_u64());
    let n = 20000;
    let (mut sum, mut sq) = (0.0, 0.0);
    for _ in 0..n {
        let g = a.gaussian();
        sum += g;
        sq += g * g;
    }
    assert!((sum / n as f64).abs() < 0.03);
    assert!((sq / n as f64 - 1.0).abs() < 0.05);
}

#[test]
fn raycast_hits_every_shape_with_correct_distance_and_normal() {
    let mut w: World<8> = World::new();
    w.push(Shape::Sphere { center: [0.0, 1.0, -5.0], radius: 1.0 }).unwrap();
    w.push(Shape::Box { center: [5.0, 1.0, 0.0], half: [1.0, 1.0, 0.5], yaw: 0.0 }).unwrap();
    w.push(Shape::Cylinder { base: [-5.0, 0.0, 0.0], radius: 0.5, height: 3.0 }).unwrap();
    w.push(Shape::Box { center: [0.0, 1.0, 6.0], half: [1.0, 1.0, 0.25], yaw: core::f64::consts::FRAC_PI_2 }).unwrap();
    let o = [0.0, 1.0, 0.0];

    let h = w.raycast(&o, &[0.0, 0.0, -1.0], 20.0).unwrap();
    assert!((h.distance - 4.0).abs() < 1e-9 && (h.normal[2] - 1.0).abs() < 1e-9);
    let h = w.raycast(&o, &[1.0, 0.0, 0.0], 20.0).unwrap();
    assert!((h.distance - 4.0).abs() < 1e-9 && (h.normal[0] + 1.0).abs() < 1e-9);
    let h = w.raycast(&o, &[-1.0, 0.0, 0.0], 20.0).unwrap();
    assert!((h.distance - 4.5).abs() < 1e-9 && (h.normal[0] - 1.0).abs() < 1e-9);
    // boîte tournée de 90° : ses demi-dimensions x et z sont échangées
    let h = w.raycast(&o, &[0.0, 0.0, 1.0], 20.0).unwrap();
    assert!((h.distance - 5.0).abs() < 1e-9, "distance {}", h.distance);
    // sol
    let h = w.raycast(&[0.0, 2.0, 3.0], &[0.0, -1.0, 0.0], 20.0).unwrap();
    assert!((h.distance - 2.0).abs() < 1e-9 && h.obstacle.is_none());
    // dessus du cylindre
    let h = w.raycast(&[-5.0, 6.0, 0.0], &[0.0, -1.0, 0.0], 20.0).unwrap();
    assert!((h.distance - 3.0).abs() < 1e-9 && (h.normal[1] - 1.0).abs() < 1e-9);
    // hors de portée
    assert!(w.raycast(&o, &[0.0, 0.0, -1.0], 3.0).is_none());
}

#[test]
fn contacts_push_out_along_the_normal() {
    let mut w: World<4> = World::new();
    w.push(Shape::Box { center: [0.0, 1.0, 0.0], half: [1.0, 1.0, 1.0], yaw: 0.3 }).unwrap();
    w.push(Shape::Cylinder { base: [6.0, 0.0, 0.0], radius: 0.5, height: 2.0 }).unwrap();
    assert!(w.contact(&[3.0, 1.0, 0.0], 0.2).is_none());
    let c = w.contact(&[1.1 * 0.3f64.cos() + 0.05, 1.0, -1.1 * 0.3f64.sin()], 0.2).unwrap();
    assert!(c.depth > 0.0 && c.depth < 0.3);
    let c = w.contact(&[5.4, 1.0, 0.0], 0.2).unwrap();
    assert!((c.normal[0] + 1.0).abs() < 1e-9 && (c.depth - 0.1).abs() < 1e-9);
    // centre dans le cylindre : profondeur = distance à la paroi + rayon
    let c = w.contact(&[5.6, 1.0, 0.0], 0.2).unwrap();
    assert!((c.depth - 0.3).abs() < 1e-9);
    // centre à l'intérieur : doit ressortir
    let c = w.contact(&[0.0, 1.0, 0.0], 0.2).unwrap();
    assert!(c.depth >= 1.0);
}

#[test]
fn steady_wind_is_resisted_and_drone_leans_into_it() {
    let mut sim = airborne(2.0);
    sim.env.wind = Wind::new([4.0, 0.0, 0.0], 0.0, 3);
    run(&mut sim, 12.0);
    let p = sim.quad.state.position();
    assert!(norm(&sub(&p, &[0.0, 2.0, 0.0])) < 0.5, "dérive {p:?}");
    let tilt = sim.quad.state.tilt();
    assert!(tilt > 0.03 && tilt < 0.25, "inclinaison {tilt}");
    // penché dans le vent : l'axe de poussée a une composante −X (contre le vent)
    assert!(sim.quad.state.thrust_axis()[0] < -0.02);
}

#[test]
fn gusty_wind_does_not_destabilize() {
    let mut sim = airborne(2.0);
    sim.env.wind = Wind::new([2.0, 0.0, -1.0], 1.5, 11);
    run(&mut sim, 40.0);
    let p = sim.quad.state.position();
    assert!(norm(&sub(&p, &[0.0, 2.0, 0.0])) < 1.0, "{p:?}");
    assert!(!sim.quad.state.crashed);
}

#[test]
fn battery_drains_at_a_realistic_rate() {
    let mut sim = airborne(2.0);
    let (e0, v0) = (sim.quad.state.battery.energy_wh, sim.quad.state.battery.voltage());
    run(&mut sim, 60.0);
    let b = sim.quad.state.battery;
    let used = e0 - b.energy_wh;
    let avg_power = used * 3600.0 / 60.0;
    assert!(avg_power > 60.0 && avg_power < 160.0, "puissance {avg_power} W");
    assert!(b.voltage() < v0 && b.voltage() > 12.0);
    assert!(b.soc() < 1.0 && b.soc() > 0.9);
}

#[test]
fn empty_battery_means_the_drone_falls() {
    let mut sim = airborne(3.0);
    sim.quad.state.battery.energy_wh = 0.0;
    run(&mut sim, 3.0);
    assert!(sim.quad.state.altitude() < 0.2);
    assert!(sim.quad.state.crashed, "impact {}", sim.quad.state.peak_impact);
}

#[test]
fn ground_effect_reduces_the_thrust_needed_near_the_floor() {
    let thrust_at = |alt: f64| {
        let mut sim = airborne(alt);
        run(&mut sim, 3.0);
        sim.quad.state.motors.iter().sum::<f64>()
    };
    let low = thrust_at(0.3);
    let high = thrust_at(3.0);
    assert!(low < high * 0.996, "bas {low} haut {high}");
}

#[test]
fn flying_into_a_pillar_collides_instead_of_passing_through() {
    let mut sim = airborne(1.5);
    sim.world.push(Shape::Cylinder { base: [4.0, 0.0, 0.0], radius: 0.4, height: 4.0 }).unwrap();
    sim.autopilot.command(Command::Goto { position: [8.0, 1.5, 0.0], yaw: 0.0 }, &sim.quad.state);
    let mut max_x: f64 = 0.0;
    for _ in 0..(10.0 / DT) as usize {
        sim.step(DT);
        max_x = max_x.max(sim.quad.state.position()[0]);
    }
    assert!(max_x < 4.0, "traversé : x max = {max_x}");
    assert!(sim.quad.state.peak_impact > 0.5);
}

#[test]
fn a_violent_impact_destroys_the_drone() {
    let mut sim = airborne(1.5);
    sim.world.push(Shape::Box { center: [3.0, 1.5, 0.0], half: [0.2, 2.0, 2.0], yaw: 0.0 }).unwrap();
    sim.quad.state.velocity = [9.0, 0.0, 0.0];
    run(&mut sim, 1.0);
    assert!(sim.quad.state.crashed);
}

#[test]
fn imu_noise_and_bias_follow_the_configuration() {
    let mut sim = Simulation::default();
    let n = 4000;
    let (mut ax, mut ay, mut gx) = (vec![], vec![], vec![]);
    for _ in 0..n {
        sim.step(DT);
        let s = sim.sensors.latest.imu;
        ax.push(s.accel[0]);
        ay.push(s.accel[1]);
        gx.push(s.gyro[0]);
    }
    let stat = |v: &Vec<f64>| {
        let m = v.iter().sum::<f64>() / v.len() as f64;
        (m, (v.iter().map(|x| (x - m) * (x - m)).sum::<f64>() / v.len() as f64).sqrt())
    };
    let (ay_m, ay_s) = stat(&ay);
    let (_, ax_s) = stat(&ax);
    let (_, gx_s) = stat(&gx);
    let c = ImuConfig::default();
    let sq = (1.0f64 / DT).sqrt();
    // au sol : l'accéléromètre mesure +g vers le haut, à un biais près
    assert!((ay_m - 9.81).abs() < 0.2, "ay {ay_m}");
    assert!((ax_s / (c.accel_noise_density * sq) - 1.0).abs() < 0.15, "ax σ {ax_s}");
    assert!((ay_s / (c.accel_noise_density * sq) - 1.0).abs() < 0.15);
    assert!((gx_s / (c.gyro_noise_density * sq) - 1.0).abs() < 0.15, "gx σ {gx_s}");
}

#[test]
fn barometer_tracks_altitude() {
    let mut sim = airborne(5.0);
    run(&mut sim, 2.0);
    let alt = sim.sensors.latest.baro.altitude;
    assert!((alt - sim.quad.state.altitude()).abs() < 0.6, "baro {alt}");
    assert!((sim.sensors.latest.baro.pressure - sim.env.atmosphere.pressure_at(40.0)).abs() < 40.0);
}

#[test]
fn gnss_is_noisy_delayed_and_geodetic() {
    let mut sim = airborne(2.0);
    sim.autopilot.command(Command::Goto { position: [6.0, 2.0, -4.0], yaw: 0.0 }, &sim.quad.state);
    run(&mut sim, 12.0);
    let fix = sim.sensors.latest.gnss.unwrap();
    let lag = sim.time - fix.timestamp;
    assert!(lag >= 0.119 && lag < 0.3, "retard {lag}");
    assert!(norm(&sub(&fix.position, &sim.quad.state.position())) < 4.0);
    // 4 m vers le nord (−Z) : latitude au-dessus de l'origine
    assert!(fix.latitude > sim.env.origin_latitude);
    assert!(fix.longitude > sim.env.origin_longitude);
    assert!(fix.satellites >= 8);
}

#[test]
fn lidar_sees_a_pillar_straight_ahead() {
    let mut sim = airborne(1.5);
    sim.world.push(Shape::Cylinder { base: [0.0, 0.0, -4.0], radius: 0.5, height: 3.0 }).unwrap();
    run(&mut sim, 1.0);
    let scan = sim.sensors.latest.lidar;
    // rayon dirigé vers l'avant (azimut 0) du canal central
    let fwd = scan.ranges[LIDAR_V / 2][LIDAR_H / 2];
    assert!((fwd - 3.5).abs() < 0.3, "avant {fwd}");
    // l'arrière ne voit rien d'autre que le sol ou rien
    let behind = scan.ranges[LIDAR_V / 2][0];
    assert!(behind > 5.0, "arrière {behind}");
    assert!(scan.nearest().unwrap() < 4.0);
    // télémètre vers le bas : ≈ altitude
    let r = sim.sensors.latest.range_down.unwrap();
    assert!((r - 1.5).abs() < 0.2);
}

#[test]
fn estimator_tracks_truth_during_a_mission() {
    let mut sim = Simulation::default();
    sim.autopilot.mission.looping = true;
    for w in [Waypoint::at(0.0, 2.0, 0.0), Waypoint::at(4.0, 2.5, -2.0), Waypoint::at(-3.0, 2.0, -4.0)] {
        sim.autopilot.mission.push(w).unwrap();
    }
    sim.autopilot.command(Command::StartMission, &sim.quad.state);
    let (mut worst_att, mut worst_pos) = (0.0f64, 0.0f64);
    let (mut est_sq, mut gnss_sq, mut n) = (0.0f64, 0.0f64, 0usize);
    for i in 0..(60.0 / DT) as usize {
        sim.step(DT);
        if i as f64 * DT > 8.0 {
            let ep = sim.estimator.position_error(&sim.quad.state);
            let gp = sim.sensors.latest.gnss.map(|f| norm(&sub(&f.position, &sim.quad.state.position()))).unwrap_or(0.0);
            worst_att = worst_att.max(sim.estimator.attitude_error(&sim.quad.state));
            worst_pos = worst_pos.max(ep);
            est_sq += ep * ep;
            gnss_sq += gp * gp;
            n += 1;
        }
    }
    let (est_rms, gnss_rms) = ((est_sq / n as f64).sqrt(), (gnss_sq / n as f64).sqrt());
    assert!(worst_att < 0.08, "attitude {worst_att} rad");
    assert!(worst_pos < 2.5, "position {worst_pos} m");
    // la fusion doit faire mieux que le GNSS seul
    assert!(est_rms < 0.85 * gnss_rms, "estimateur {est_rms} m, GNSS {gnss_rms} m");
}

#[test]
fn flying_on_estimates_alone_works() {
    let mut sim = Simulation::default();
    sim.use_estimator = true;
    sim.autopilot.command(Command::Takeoff { altitude: 2.0 }, &sim.quad.state);
    run(&mut sim, 8.0);
    sim.autopilot.command(Command::Goto { position: [3.0, 2.0, -3.0], yaw: 0.5 }, &sim.control_state());
    run(&mut sim, 12.0);
    let err = norm(&sub(&sim.quad.state.position(), &[3.0, 2.0, -3.0]));
    assert!(err < 1.2, "erreur {err}");
    assert!(!sim.quad.state.crashed);
    assert!(sim.quad.state.tilt() < 0.15);
}

#[test]
fn same_seed_same_flight_different_seed_different_noise() {
    let fly = |seed: u64| {
        let mut s = Simulation::new(QuadParams::default(), Gains::default(), seed);
        s.env.wind = Wind::new([1.0, 0.0, 0.0], 1.0, seed);
        s.autopilot.command(Command::Takeoff { altitude: 2.0 }, &s.quad.state);
        run(&mut s, 5.0);
        (s.quad.state.position(), s.sensors.latest.imu.gyro)
    };
    assert_eq!(fly(9), fly(9));
    assert_ne!(fly(9).1, fly(10).1);
}

#[test]
fn mission_never_stalls_between_waypoints() {
    // Régression : le point lièvre ne doit jamais laisser le drone s'immobiliser à mi-chemin.
    for (wind, seed) in [([0.0, 0.0, 0.0], 1), ([1.5, 0.0, 0.5], 2), ([-2.0, 0.0, 1.0], 3)] {
        let mut sim = Simulation::new(QuadParams::default(), Gains::default(), seed);
        sim.env.wind = Wind::new(wind, 0.0, seed);
        sim.autopilot.mission.looping = true;
        for w in [Waypoint::at(0.0, 2.0, 0.0), Waypoint::at(5.0, 2.0, 0.0), Waypoint::at(5.0, 2.0, -5.0), Waypoint::at(0.0, 2.0, -5.0)] {
            sim.autopilot.mission.push(w).unwrap();
        }
        sim.autopilot.command(Command::StartMission, &sim.quad.state);
        let (mut visits, mut last, mut slow_for) = (0, usize::MAX, 0.0);
        for _ in 0..(90.0 / DT) as usize {
            sim.step(DT);
            let i = sim.autopilot.mission.current_index();
            if i != last {
                visits += 1;
                last = i;
            }
            slow_for = if sim.autopilot.mode() == FlightMode::Mission && sim.quad.state.speed() < 0.15 { slow_for + DT } else { 0.0 };
            assert!(slow_for < 3.0, "immobilisé {slow_for:.1} s à t={:.1} (point {i})", sim.time);
        }
        assert!(visits >= 12, "seulement {visits} points validés en 90 s");
    }
}
