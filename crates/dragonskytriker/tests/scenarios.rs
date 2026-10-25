use dragonskytriker::scenario::Scenario;
use dragonskytriker_core::geometry::linalg::{norm, sub};
use dragonskytriker_core::{Command, FlightMode, Gains, QuadParams, Simulation};

const DT: f64 = 1.0 / 240.0;

fn world(name: &str) -> Scenario {
    Scenario::load(&format!("{}/assets/worlds/{name}.json", env!("CARGO_MANIFEST_DIR"))).unwrap()
}

fn fly(name: &str, estimator: bool, seconds: f64) -> Simulation {
    let mut sim = Simulation::new(QuadParams::default(), Gains::default(), 4);
    sim.use_estimator = estimator;
    world(name).apply(&mut sim);
    let st = sim.control_state();
    sim.autopilot.command(Command::StartMission, &st);
    for _ in 0..(seconds / DT) as usize {
        sim.step(DT);
        if sim.quad.state.crashed {
            break;
        }
    }
    sim
}

#[test]
fn scenario_survives_a_json_round_trip() {
    let s = Scenario::default_scene();
    let json = serde_json::to_string(&s).unwrap();
    let back: Scenario = serde_json::from_str(&json).unwrap();
    assert_eq!(back.obstacles, s.obstacles);
    assert_eq!(back.mission.waypoints.len(), s.mission.waypoints.len());
}

#[test]
fn studio_world_is_flown_without_a_scratch_on_truth() {
    let sim = fly("studio", false, 90.0);
    assert!(!sim.quad.state.crashed);
    assert_eq!(sim.autopilot.mode(), FlightMode::Mission);
    assert!(sim.quad.state.peak_impact < 0.5, "choc {}", sim.quad.state.peak_impact);
}

#[test]
fn slalom_is_flown_through_the_pillars_on_truth() {
    let sim = fly("slalom", false, 80.0);
    assert!(!sim.quad.state.crashed);
    assert!(sim.quad.state.peak_impact < 0.5, "choc {}", sim.quad.state.peak_impact);
    let end = [22.0, 1.8, 0.0];
    assert!(norm(&sub(&sim.quad.state.position(), &end)) < 1.5, "{:?}", sim.quad.state.position());
}

#[test]
fn slalom_is_flown_on_sensors_alone() {
    let sim = fly("slalom", true, 80.0);
    assert!(!sim.quad.state.crashed, "crash, impact {}", sim.quad.state.peak_impact);
    let end = [22.0, 1.8, 0.0];
    assert!(norm(&sub(&sim.quad.state.position(), &end)) < 2.5, "{:?}", sim.quad.state.position());
}
