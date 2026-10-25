//! Batterie LiPo 4S : tension en charge, chute avec la décharge, poussée disponible.

/// Batterie simple : tension à vide selon l'état de charge + résistance interne.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Battery {
    /// Énergie totale (Wh).
    pub capacity_wh: f64,
    /// Énergie restante (Wh).
    pub energy_wh: f64,
    /// Résistance interne du pack (Ω).
    pub resistance: f64,
    /// Nombre de cellules en série.
    pub cells: f64,
    voltage: f64,
}

impl Default for Battery {
    /// 4S 3000 mAh ≈ 44 Wh.
    fn default() -> Self {
        let mut b = Self {
            capacity_wh: 44.4,
            energy_wh: 44.4,
            resistance: 0.025,
            cells: 4.0,
            voltage: 16.8,
        };
        b.voltage = b.open_circuit_voltage();
        b
    }
}

impl Battery {
    /// État de charge, 0 à 1.
    pub fn soc(&self) -> f64 {
        (self.energy_wh / self.capacity_wh).clamp01()
    }

    /// Tension à vide du pack (V) : 4,2 V/cellule pleine, plateau, chute en fin de charge.
    pub fn open_circuit_voltage(&self) -> f64 {
        let s = self.soc();
        let per_cell = 3.30 + 0.60 * s + 0.30 * s * s * s * s;
        per_cell * self.cells
    }

    /// Tension en charge (V).
    #[inline]
    pub fn voltage(&self) -> f64 {
        self.voltage
    }

    /// Vrai si la batterie est vide.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.energy_wh <= 0.0
    }

    /// Facteur multiplicatif de la poussée maximale (∝ V²).
    pub fn thrust_factor(&self) -> f64 {
        let nominal = 3.8 * self.cells;
        let r = self.voltage / nominal;
        (r * r).clamp(0.0, 1.15)
    }

    /// Consomme `power_w` pendant `dt`, met à jour la tension en charge.
    pub fn step(&mut self, power_w: f64, dt: f64) {
        self.energy_wh = (self.energy_wh - power_w * dt / 3600.0).max(0.0);
        let v_oc = self.open_circuit_voltage();
        // I ≈ P / V, avec une chute I·R (une itération suffit).
        let i = power_w / v_oc.max(1.0);
        self.voltage = (v_oc - i * self.resistance).max(0.0);
        if self.is_empty() {
            self.voltage = 0.0;
        }
    }
}

trait Clamp01 {
    fn clamp01(self) -> f64;
}
impl Clamp01 for f64 {
    #[inline]
    fn clamp01(self) -> f64 {
        if self < 0.0 {
            0.0
        } else if self > 1.0 {
            1.0
        } else {
            self
        }
    }
}
