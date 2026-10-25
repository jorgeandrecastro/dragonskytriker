//! Simulation sans fenêtre, plus rapide que le temps réel, avec export CSV.
//!
//! ```text
//! cargo run --release -p dragonskytriker-core --example headless -- [secondes] [sortie.csv] [graine]
//! ```

use dragonskytriker_core::geometry::linalg::{norm, sub};
use dragonskytriker_core::*;
use std::io::Write;
use std::time::Instant;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let seconds: f64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(120.0);
    let out = args.get(2).cloned().unwrap_or_else(|| "flight_log.csv".into());
    let seed: u64 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(1);

    let dt = 1.0 / 240.0;
    let mut sim = Simulation::new(QuadParams::default(), Gains::default(), seed);
    sim.use_estimator = true; // le drone vole sur ses capteurs, pas sur la vérité
    sim.env.wind = Wind::new([1.5, 0.0, -0.8], 1.0, seed);
    for (x, z) in [(6.5, 1.5), (-6.5, -4.0), (0.0, -7.5), (5.0, -6.0)] {
        sim.world.push(Shape::Cylinder { base: [x, 0.0, z], radius: 0.4, height: 4.0 }).unwrap();
    }
    sim.autopilot.mission.looping = true;
    let n = 12;
    for i in 0..n {
        let t = core::f64::consts::TAU * i as f64 / n as f64;
        let _ = sim.autopilot.mission.push(Waypoint::at(4.5 * t.sin(), 2.1 + 0.5 * t.sin(), -3.0 * (2.0 * t).sin()));
    }
    sim.autopilot.command(Command::StartMission, &sim.quad.state);

    let mut file = std::io::BufWriter::new(std::fs::File::create(&out).expect("création du CSV"));
    writeln!(file, "t,x,y,z,est_x,est_y,est_z,speed,tilt_deg,soc,voltage,power_w,att_err_deg,pos_err_m,nearest_lidar_m").unwrap();

    let start = Instant::now();
    let steps = (seconds / dt) as usize;
    let (mut worst_att, mut sum_pos) = (0.0f64, 0.0f64);
    for i in 0..steps {
        sim.step(dt);
        let q = &sim.quad.state;
        let att = sim.estimator.attitude_error(q).to_degrees();
        let pos = sim.estimator.position_error(q);
        worst_att = worst_att.max(att);
        sum_pos += pos;
        if i % 24 == 0 {
            let e = sim.estimator.position();
            let p = q.position();
            writeln!(
                file,
                "{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.2},{:.4},{:.2},{:.1},{:.2},{:.3},{:.2}",
                sim.time, p[0], p[1], p[2], e[0], e[1], e[2], q.speed(), q.tilt().to_degrees(),
                q.battery.soc(), q.battery.voltage(), q.power, att, pos,
                sim.sensors.latest.lidar.nearest().unwrap_or(f32::NAN)
            )
            .unwrap();
        }
        if sim.quad.state.crashed {
            println!("Crash à t = {:.1} s (impact {:.1} m/s)", sim.time, sim.quad.state.peak_impact);
            break;
        }
    }
    let wall = start.elapsed().as_secs_f64();
    let q = &sim.quad.state;
    println!("{:.0} s simulées en {:.2} s : ×{:.0} le temps réel", sim.time, wall, sim.time / wall);
    println!("batterie : {:.0} % ({:.1} V), puissance moyenne ≈ {:.0} W", q.battery.soc() * 100.0, q.battery.voltage(), (44.4 * (1.0 - q.battery.soc())) * 3600.0 / sim.time);
    println!("estimation : attitude max {:.1}°, position moyenne {:.2} m", worst_att, sum_pos / steps as f64);
    println!("écart à la consigne : {:.2} m", norm(&sub(&q.position(), &sim.autopilot.setpoint().position)));
    println!("journal : {out}");
}
