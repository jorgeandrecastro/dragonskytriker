use std::f64::consts::PI;
use tiny_rodriguez_nostd::linalg::{det, dot, norm};
use tiny_rodriguez_nostd::{Pose3, Rotation3, Vec3};

/// Générateur pseudo-aléatoire minimal (xorshift64*), sans dépendance.
struct Rng(u64);
impl Rng {
    fn next_u64(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545F4914F6CDD1D)
    }
    fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.unit()
    }
    /// Vecteur de rotation, angles proches de 0 et de π sur-représentés.
    fn omega(&mut self) -> Vec3 {
        let axis = [self.range(-1.0, 1.0), self.range(-1.0, 1.0), self.range(-1.0, 1.0)];
        let n = norm(&axis);
        if n < 1e-6 {
            return [0.0, 0.0, 0.0];
        }
        let theta = match self.next_u64() % 4 {
            0 => self.range(0.0, 1e-3),
            1 => PI - self.range(0.0, 1e-3),
            _ => self.range(0.0, PI),
        };
        [axis[0] / n * theta, axis[1] / n * theta, axis[2] / n * theta]
    }
}

fn mat_close(a: &[[f64; 3]; 3], b: &[[f64; 3]; 3], tol: f64) -> bool {
    (0..3).all(|i| (0..3).all(|j| (a[i][j] - b[i][j]).abs() <= tol))
}
fn vec_close(a: &Vec3, b: &Vec3, tol: f64) -> bool {
    (0..3).all(|i| (a[i] - b[i]).abs() <= tol)
}

const N: usize = 5000;

#[test]
fn exp_is_always_valid() {
    let mut rng = Rng(1);
    for _ in 0..N {
        assert!(Rotation3::exp(&rng.omega()).is_valid(1e-12));
    }
}

#[test]
fn exp_log_roundtrip_including_near_pi() {
    let mut rng = Rng(2);
    for _ in 0..N {
        let w = rng.omega();
        let r = Rotation3::exp(&w);
        let r2 = Rotation3::exp(&r.log());
        assert!(mat_close(&r.matrix, &r2.matrix, 1e-9), "w = {w:?}");
        let th = norm(&w);
        // log est de norme égale à l'angle (loin de π, le signe du log est unique)
        assert!((norm(&r.log()) - th).abs() < 1e-8, "w = {w:?}");
        if th < PI - 1e-2 {
            assert!(vec_close(&r.log(), &w, 1e-8), "w = {w:?}");
        }
    }
}

#[test]
fn log_at_exactly_pi_for_many_axes() {
    let axes: [Vec3; 9] = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 1.0],
        [1.0, 0.0, 1.0],
        [1.0, 1.0, 1.0],
        [1.0, -2.0, 3.0],
        [-0.3, 0.2, -0.9],
    ];
    for ax in axes {
        let r = Rotation3::from_axis_angle(&ax, PI);
        let l = r.log();
        assert!((norm(&l) - PI).abs() < 1e-7, "axis {ax:?}");
        let r2 = Rotation3::exp(&l);
        assert!(mat_close(&r.matrix, &r2.matrix, 1e-7), "axis {ax:?}");
        assert!((r.angle() - PI).abs() < 1e-7);
    }
}

#[test]
fn hat_vee_roundtrip() {
    let w = [0.3, -1.2, 2.5];
    assert_eq!(Rotation3::vee(&Rotation3::hat(&w)), w);
}

#[test]
fn composition_is_associative_and_preserves_norm() {
    let mut rng = Rng(3);
    for _ in 0..N {
        let (a, b, c) = (
            Rotation3::exp(&rng.omega()),
            Rotation3::exp(&rng.omega()),
            Rotation3::exp(&rng.omega()),
        );
        let l = (a * b) * c;
        let r = a * (b * c);
        assert!(mat_close(&l.matrix, &r.matrix, 1e-12));
        let v = [rng.range(-5.0, 5.0), rng.range(-5.0, 5.0), rng.range(-5.0, 5.0)];
        assert!((norm(&(a * v)) - norm(&v)).abs() < 1e-12);
    }
}

#[test]
fn quaternion_roundtrip_and_canonical_form() {
    let mut rng = Rng(4);
    for _ in 0..N {
        let r = Rotation3::exp(&rng.omega());
        let q = r.to_quaternion();
        assert!(q[3] >= 0.0);
        let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        assert!((n - 1.0).abs() < 1e-12);
        assert!(mat_close(&r.matrix, &Rotation3::from_quaternion(&q).matrix, 1e-12));
        // invariance au signe et à l'échelle
        let neg = [-q[0], -q[1], -q[2], -q[3]];
        let big = [3.0 * q[0], 3.0 * q[1], 3.0 * q[2], 3.0 * q[3]];
        assert!(mat_close(&r.matrix, &Rotation3::from_quaternion(&neg).matrix, 1e-12));
        assert!(mat_close(&r.matrix, &Rotation3::from_quaternion(&big).matrix, 1e-12));
    }
}

#[test]
fn degenerate_inputs() {
    assert_eq!(Rotation3::from_axis_angle(&[0.0; 3], 1.0), Rotation3::identity());
    assert_eq!(Rotation3::from_quaternion(&[0.0; 4]), Rotation3::identity());
    assert_eq!(Rotation3::default(), Rotation3::identity());
    // NaN / infini : pas de panique
    let nan = f64::NAN;
    let _ = Rotation3::exp(&[nan, 0.0, 0.0]).log();
    let _ = Rotation3::exp(&[f64::INFINITY, 0.0, 0.0]).to_quaternion();
    let _ = Rotation3::exp(&[nan; 3]).renormalized();
    assert!(!Rotation3::exp(&[nan; 3]).is_valid(1e-9));
}

#[test]
fn geodesic_distance_and_interpolation() {
    let mut rng = Rng(5);
    for _ in 0..N {
        let a = Rotation3::exp(&rng.omega());
        let b = Rotation3::exp(&rng.omega());
        let d = a.geodesic_distance(&b);
        assert!((d - b.geodesic_distance(&a)).abs() < 1e-9);
        assert!(mat_close(&a.interpolate(&b, 0.0).matrix, &a.matrix, 1e-9));
        // éviter le cas t=1 près de π où log est ambigu mais exp reste exact
        assert!(mat_close(&a.interpolate(&b, 1.0).matrix, &b.matrix, 1e-8));
        let m = a.interpolate(&b, 0.5);
        assert!((a.geodesic_distance(&m) - d / 2.0).abs() < 1e-7);
    }
}

#[test]
fn jacobian_times_inverse_is_identity() {
    let mut rng = Rng(6);
    for _ in 0..N {
        let w = rng.omega();
        if norm(&w) > 2.0 * PI - 0.5 {
            continue;
        }
        let j = Rotation3::left_jacobian(&w);
        let ji = Rotation3::left_jacobian_inv(&w);
        let p = tiny_rodriguez_nostd::linalg::mat_mul(&j, &ji);
        assert!(mat_close(&p, &tiny_rodriguez_nostd::MAT3_IDENTITY, 1e-9), "w = {w:?}");
    }
}

#[test]
fn constant_velocity_integration_is_exact() {
    let w = [0.5, 1.2, 0.3];
    let (dt, n) = (0.016, 1000);
    let mut r = Rotation3::identity();
    for _ in 0..n {
        r = r.integrate(&w, dt);
    }
    let exact = Rotation3::exp(&[w[0] * dt * n as f64, w[1] * dt * n as f64, w[2] * dt * n as f64]);
    assert!(mat_close(&r.matrix, &exact.matrix, 1e-10));
}

#[test]
fn body_and_world_integration_are_conjugate() {
    let mut rng = Rng(7);
    for _ in 0..N {
        let r = Rotation3::exp(&rng.omega());
        let w = [rng.range(-3.0, 3.0), rng.range(-3.0, 3.0), rng.range(-3.0, 3.0)];
        let body = r.integrate(&w, 0.01);
        let world = r.integrate_world(&r.rotate(&w), 0.01);
        assert!(mat_close(&body.matrix, &world.matrix, 1e-12));
    }
}

#[test]
fn drift_is_bounded_and_renormalization_fixes_it() {
    let w = [1.3, -0.7, 2.1];
    let mut r = Rotation3::identity();
    for i in 0..100_000 {
        r = r.integrate(&w, 0.001);
        if i % 500 == 0 {
            r = r.renormalized();
        }
    }
    assert!(r.orthogonality_error() < 1e-12);
    assert!((det(&r.matrix) - 1.0).abs() < 1e-12);

    // matrice perturbée → reprojetée sur SO(3)
    let mut rng = Rng(8);
    for _ in 0..1000 {
        let mut p = Rotation3::exp(&rng.omega());
        for i in 0..3 {
            for j in 0..3 {
                p.matrix[i][j] += rng.range(-1e-3, 1e-3);
            }
        }
        assert!(p.renormalized().is_valid(1e-12));
    }
}

#[test]
fn se3_exp_log_roundtrip() {
    let mut rng = Rng(9);
    for _ in 0..N {
        let w = rng.omega();
        if norm(&w) > PI - 1e-2 {
            continue;
        }
        let xi = [rng.range(-5.0, 5.0), rng.range(-5.0, 5.0), rng.range(-5.0, 5.0), w[0], w[1], w[2]];
        let back = Pose3::exp(&xi).log();
        for k in 0..6 {
            assert!((back[k] - xi[k]).abs() < 1e-7, "xi = {xi:?}, back = {back:?}");
        }
    }
}

#[test]
fn se3_inverse_associativity_and_transform() {
    let mut rng = Rng(10);
    let mk = |rng: &mut Rng| {
        Pose3::new(
            Rotation3::exp(&rng.omega()),
            [rng.range(-9.0, 9.0), rng.range(-9.0, 9.0), rng.range(-9.0, 9.0)],
        )
    };
    for _ in 0..N {
        let (a, b, c) = (mk(&mut rng), mk(&mut rng), mk(&mut rng));
        let id = a.compose(&a.inverse());
        assert!(mat_close(&id.rotation.matrix, &Rotation3::identity().matrix, 1e-12));
        assert!(vec_close(&id.translation, &[0.0; 3], 1e-11));
        let l = a.compose(&b).compose(&c);
        let r = a.compose(&b.compose(&c));
        assert!(mat_close(&l.rotation.matrix, &r.rotation.matrix, 1e-12));
        assert!(vec_close(&l.translation, &r.translation, 1e-10));
        let p = [rng.range(-3.0, 3.0), rng.range(-3.0, 3.0), rng.range(-3.0, 3.0)];
        assert!(vec_close(
            &a.compose(&b).transform_point(&p),
            &a.transform_point(&b.transform_point(&p)),
            1e-10
        ));
        // isométrie
        let q = [rng.range(-3.0, 3.0), rng.range(-3.0, 3.0), rng.range(-3.0, 3.0)];
        let d0 = norm(&[p[0] - q[0], p[1] - q[1], p[2] - q[2]]);
        let (pa, qa) = (a.transform_point(&p), a.transform_point(&q));
        let d1 = norm(&[pa[0] - qa[0], pa[1] - qa[1], pa[2] - qa[2]]);
        assert!((d0 - d1).abs() < 1e-11);
    }
    let _ = dot(&[1.0; 3], &[1.0; 3]);
}

#[test]
fn se3_constant_twist_integration_is_exact() {
    let xi = [1.0, 0.0, 0.0, 0.0, 0.0, 0.5];
    let mut pose = Pose3::identity();
    for _ in 0..1000 {
        pose = pose.integrate(&xi, 0.001);
    }
    let exact = Pose3::exp(&xi);
    assert!(mat_close(&pose.rotation.matrix, &exact.rotation.matrix, 1e-12));
    assert!(vec_close(&pose.translation, &exact.translation, 1e-10));
    // vol dans le repère corps : demi-tour de lacet à vitesse 1 m/s avant
    let circle = Pose3::exp(&[1.0, 0.0, 0.0, 0.0, 0.0, 1.0]);
    assert!((norm(&circle.translation) - 2.0 * (0.5f64).sin()).abs() < 1e-12);
}

#[test]
fn batch_in_place_matches_sequential_loop() {
    let mut rng = Rng(11);
    let n = 300;
    let rots: Vec<Rotation3> = (0..n).map(|_| Rotation3::exp(&rng.omega())).collect();
    let oms: Vec<Vec3> = (0..n).map(|_| rng.omega()).collect();
    let mut batch = rots.clone();
    Rotation3::integrate_batch_in_place(&mut batch, &oms, 0.01);
    for i in 0..n {
        assert_eq!(batch[i], rots[i].integrate(&oms[i], 0.01));
    }
    let mut poses: Vec<Pose3> = (0..n).map(|i| Pose3::new(rots[i], [i as f64, 0.0, 1.0])).collect();
    let tw: Vec<[f64; 6]> = (0..n).map(|i| [1.0, 0.0, 0.0, oms[i][0], oms[i][1], oms[i][2]]).collect();
    let reference: Vec<Pose3> = poses.iter().zip(&tw).map(|(p, t)| p.integrate(t, 0.01)).collect();
    Pose3::integrate_batch_in_place(&mut poses, &tw, 0.01);
    assert_eq!(poses, reference);
}

#[test]
#[should_panic]
fn batch_length_mismatch_panics() {
    let mut r = [Rotation3::identity(); 3];
    Rotation3::integrate_batch_in_place(&mut r, &[[0.0; 3]; 2], 0.1);
}

#[cfg(feature = "alloc")]
#[test]
fn alloc_batch_matches_in_place() {
    let mut rng = Rng(12);
    let rots: Vec<Rotation3> = (0..100).map(|_| Rotation3::exp(&rng.omega())).collect();
    let oms: Vec<Vec3> = (0..100).map(|_| rng.omega()).collect();
    let out = Rotation3::integrate_batch(&rots, &oms, 0.02);
    let mut inplace = rots.clone();
    Rotation3::integrate_batch_in_place(&mut inplace, &oms, 0.02);
    assert_eq!(out, inplace);
}
