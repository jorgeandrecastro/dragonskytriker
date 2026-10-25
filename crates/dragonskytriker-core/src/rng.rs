//! Générateur pseudo-aléatoire déterministe (SplitMix64) : mêmes graines, mêmes vols.

/// Générateur pseudo-aléatoire minimal, sans allocation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rng {
    state: u64,
}

impl Rng {
    /// Crée un générateur à partir d'une graine.
    pub const fn new(seed: u64) -> Self {
        Self { state: seed.wrapping_add(0x9E37_79B9_7F4A_7C15) }
    }

    /// Entier 64 bits uniforme.
    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Réel uniforme dans [0, 1).
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Réel gaussien centré réduit (Box-Muller).
    pub fn gaussian(&mut self) -> f64 {
        let u1 = 1.0 - self.uniform(); // ]0, 1]
        let u2 = self.uniform();
        libm::sqrt(-2.0 * libm::log(u1)) * libm::cos(core::f64::consts::TAU * u2)
    }

    /// Vecteur 3D de gaussiennes d'écart-type `sigma`.
    pub fn gaussian3(&mut self, sigma: f64) -> [f64; 3] {
        [self.gaussian() * sigma, self.gaussian() * sigma, self.gaussian() * sigma]
    }
}
