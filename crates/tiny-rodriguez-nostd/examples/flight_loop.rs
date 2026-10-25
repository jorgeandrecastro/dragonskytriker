//! Boucle de vol simplifiée : intègre la pose d'un drone à 1 kHz.
//! Le code de la bibliothèque est `no_std` ; cet exemple utilise `std` pour afficher.

use tiny_rodriguez_nostd::{Pose3, Rotation3};

fn main() {
    let dt = 0.001;
    let mut pose = Pose3::identity();

    // Torseur corps [vx, vy, vz, wx, wy, wz] : 1 m/s vers l'avant (−Z), lacet de 0,5 rad/s.
    let twist = [0.0, 0.0, -1.0, 0.0, 0.5, 0.0];

    for step in 0..5000 {
        pose = pose.integrate(&twist, dt);
        // Contrôle de la dérive numérique toutes les 500 itérations.
        if step % 500 == 0 {
            pose.rotation = pose.rotation.renormalized();
        }
    }

    let q = pose.rotation.to_quaternion();
    println!("position   = {:?}", pose.translation);
    println!("quaternion = {:?}", q);
    println!("dérive     = {:e}", pose.rotation.orthogonality_error());

    // Hélice : rotation autour de Y à 300 rad/s.
    let mut prop = Rotation3::identity();
    for _ in 0..100 {
        prop = prop.integrate(&[0.0, 300.0, 0.0], dt);
    }
    println!("angle hélice = {:.3} rad", prop.angle());
}
