use dragonskytriker_core::geometry::linalg::{norm, sub};
use dragonskytriker_core::{
    Autopilot, Command, FlightMode, Gains, Mixer, QuadParams, Quadrotor, Waypoint, GRAVITY,
};

const DT: f64 = 1.0 / 240.0;

fn fly(ap: &mut Autopilot<16>, quad: &mut Quadrotor, seconds: f64) {
    let n = (seconds / DT) as usize;
    for _ in 0..n {
        let m = ap.update(&quad.state, DT);
        quad.step(&m, DT);
    }
}

fn setup() -> (Autopilot<16>, Quadrotor) {
    let p = QuadParams::default();
    (Autopilot::new(p, Gains::default()), Quadrotor::new(p))
}

#[test]
fn mixer_roundtrip() {
    let p = QuadParams::default();
    let mx = Mixer::new(&p);
    for (t, tau) in [
        (8.0, [0.0, 0.0, 0.0]),
        (10.0, [0.1, -0.02, 0.05]),
        (12.0, [-0.2, 0.03, 0.1]),
    ] {
        let f = mx.allocate(t, &tau);
        let (t2, tau2) = mx.wrench(&f);
        assert!((t - t2).abs() < 1e-9, "{t} vs {t2}");
        for k in 0..3 {
            assert!((tau[k] - tau2[k]).abs() < 1e-9, "axe {k}");
        }
    }
}

#[test]
fn disarmed_drone_stays_on_ground() {
    let (mut ap, mut quad) = setup();
    fly(&mut ap, &mut quad, 2.0);
    assert!(quad.state.on_ground);
    assert_eq!(quad.state.position(), [0.0; 3]);
    assert_eq!(ap.mode(), FlightMode::Disarmed);
}

#[test]
fn takeoff_reaches_altitude_and_hovers() {
    let (mut ap, mut quad) = setup();
    ap.command(Command::Takeoff { altitude: 2.0 }, &quad.state);
    fly(&mut ap, &mut quad, 6.0);
    assert_eq!(ap.mode(), FlightMode::Hover);
    let s = &quad.state;
    assert!((s.altitude() - 2.0).abs() < 0.05, "alt = {}", s.altitude());
    assert!(s.speed() < 0.05, "speed = {}", s.speed());
    assert!(s.tilt() < 0.02, "tilt = {}", s.tilt());
    assert!(s.pose.rotation.is_valid(1e-9));
    // en vol stationnaire, chaque moteur porte ≈ le quart du poids
    let hover = quad.params.hover_thrust_per_motor();
    for m in s.motors {
        assert!((m - hover).abs() < 0.2, "moteur {m} vs {hover}");
    }
}

#[test]
fn goto_converges_with_small_tilt_and_no_overshoot() {
    let (mut ap, mut quad) = setup();
    ap.command(Command::Takeoff { altitude: 1.5 }, &quad.state);
    fly(&mut ap, &mut quad, 5.0);
    ap.command(Command::Goto { position: [4.0, 1.5, -3.0], yaw: 0.0 }, &quad.state);
    let mut max_tilt: f64 = 0.0;
    let mut min_alt: f64 = 9.0;
    for _ in 0..(10.0 / DT) as usize {
        let m = ap.update(&quad.state, DT);
        quad.step(&m, DT);
        max_tilt = max_tilt.max(quad.state.tilt());
        min_alt = min_alt.min(quad.state.altitude());
    }
    let err = norm(&sub(&quad.state.position(), &[4.0, 1.5, -3.0]));
    assert!(err < 0.05, "erreur finale {err}");
    assert!(max_tilt <= Gains::default().max_tilt + 0.05, "tilt {max_tilt}");
    assert!(min_alt > 1.2, "perte d'altitude {min_alt}");
}

#[test]
fn yaw_tracking_including_wraparound() {
    let (mut ap, mut quad) = setup();
    ap.command(Command::Takeoff { altitude: 1.5 }, &quad.state);
    fly(&mut ap, &mut quad, 5.0);
    for yaw in [1.0_f64, 2.8, -2.8, -1.0, 0.0] {
        ap.command(Command::Goto { position: [0.0, 1.5, 0.0], yaw }, &quad.state);
        fly(&mut ap, &mut quad, 4.0);
        let mut d = quad.state.yaw() - yaw;
        while d > core::f64::consts::PI {
            d -= 2.0 * core::f64::consts::PI;
        }
        while d < -core::f64::consts::PI {
            d += 2.0 * core::f64::consts::PI;
        }
        assert!(d.abs() < 0.03, "yaw {yaw}: erreur {d}");
        assert!(norm(&sub(&quad.state.position(), &[0.0, 1.5, 0.0])) < 0.1);
    }
}

#[test]
fn mission_visits_all_waypoints_then_hovers() {
    let (mut ap, mut quad) = setup();
    let pts = [
        Waypoint::at(0.0, 1.5, 0.0),
        Waypoint::at(5.0, 2.0, 0.0),
        Waypoint::at(5.0, 2.5, -5.0).with_yaw(1.0),
        Waypoint::at(0.0, 1.5, -5.0).with_hold(1.0),
    ];
    for w in pts {
        ap.mission.push(w).unwrap();
    }
    // La mission valide un survol jusqu'à 2×radius et son point lièvre coupe les virages :
    // un point est « visité » dans ce rayon de franchissement, plus une marge.
    let tol = ap.mission.radius * 2.0 + 0.5;
    ap.command(Command::StartMission, &quad.state);
    assert_eq!(ap.mode(), FlightMode::Takeoff);
    let mut visited = [false; 4];
    for _ in 0..(60.0 / DT) as usize {
        let m = ap.update(&quad.state, DT);
        quad.step(&m, DT);
        for (i, w) in pts.iter().enumerate() {
            if norm(&sub(&quad.state.position(), &w.position)) < tol {
                visited[i] = true;
            }
        }
        if ap.mode() == FlightMode::Hover {
            break;
        }
    }
    assert!(visited.iter().all(|v| *v), "points visités : {visited:?}");
    assert_eq!(ap.mode(), FlightMode::Hover);
    fly(&mut ap, &mut quad, 4.0);
    assert!(norm(&sub(&quad.state.position(), &pts[3].position)) < 0.1);
    assert!(quad.state.altitude() > 1.0, "ne doit pas tomber");
}

#[test]
fn looping_mission_keeps_flying() {
    let (mut ap, mut quad) = setup();
    ap.mission.looping = true;
    for w in [Waypoint::at(0.0, 2.0, 0.0), Waypoint::at(4.0, 2.0, 0.0), Waypoint::at(4.0, 2.0, -4.0)] {
        ap.mission.push(w).unwrap();
    }
    ap.command(Command::StartMission, &quad.state);
    fly(&mut ap, &mut quad, 60.0);
    assert_eq!(ap.mode(), FlightMode::Mission);
    assert!(quad.state.altitude() > 1.5);
    assert!(quad.state.pose.rotation.is_valid(1e-9));
}

#[test]
fn landing_ends_disarmed_on_ground() {
    let (mut ap, mut quad) = setup();
    ap.command(Command::Takeoff { altitude: 2.0 }, &quad.state);
    fly(&mut ap, &mut quad, 6.0);
    ap.command(Command::Land, &quad.state);
    fly(&mut ap, &mut quad, 8.0);
    assert!(quad.state.on_ground);
    assert_eq!(ap.mode(), FlightMode::Disarmed);
    assert!(quad.state.altitude().abs() < 1e-9);
}

#[test]
fn survives_a_hard_kick() {
    let (mut ap, mut quad) = setup();
    ap.command(Command::Takeoff { altitude: 3.0 }, &quad.state);
    fly(&mut ap, &mut quad, 6.0);
    // choc : forte vitesse angulaire et latérale
    quad.state.omega = [4.0, 1.0, -3.0];
    quad.state.velocity = [2.0, -1.0, 1.0];
    fly(&mut ap, &mut quad, 8.0);
    assert!(norm(&sub(&quad.state.position(), &[0.0, 3.0, 0.0])) < 0.2);
    assert!(quad.state.tilt() < 0.05);
}

#[test]
fn rotation_stays_on_so3_over_a_long_flight() {
    let (mut ap, mut quad) = setup();
    ap.mission.looping = true;
    for w in [
        Waypoint::at(0.0, 2.0, 0.0),
        Waypoint::at(3.0, 3.0, -3.0).with_yaw(1.5),
        Waypoint::at(-3.0, 2.0, -3.0).with_yaw(-1.5),
    ] {
        ap.mission.push(w).unwrap();
    }
    ap.command(Command::StartMission, &quad.state);
    fly(&mut ap, &mut quad, 120.0);
    assert!(quad.state.pose.rotation.orthogonality_error() < 1e-9);
    // la poussée de vol stationnaire est bien celle du poids
    assert!((quad.params.hover_thrust_per_motor() * 4.0 - quad.params.mass * GRAVITY).abs() < 1e-12);
}
