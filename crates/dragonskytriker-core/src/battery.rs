//! Batterie LiPo : tension en charge, chute avec la décharge, poussée disponible.

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
    /// Tension nominale d'une cellule LiPo (V), pour convertir mAh en Wh.
    pub const NOMINAL_CELL_V: f64 = 3.7;

    /// Pack de `cells` cellules, `capacity_mah` mAh, résistance interne `resistance` (Ω),
    /// chargé à `charge` (0 à 1).
    pub fn new(cells: u32, capacity_mah: f64, resistance: f64, charge: f64) -> Self {
        let cells = cells as f64;
        let capacity_wh = capacity_mah / 1000.0 * Self::NOMINAL_CELL_V * cells;
        let mut b = Self {
            capacity_wh,
            energy_wh: capacity_wh * charge.clamp01(),
            resistance,
            cells,
            voltage: 0.0,
        };
        b.voltage = b.open_circuit_voltage();
        b
    }

    /// Capacité en mAh.
    pub fn capacity_mah(&self) -> f64 {
        self.capacity_wh / (Self::NOMINAL_CELL_V * self.cells) * 1000.0
    }

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_matches_default_pack() {
        let b = Battery::new(4, 3000.0, 0.025, 1.0);
        assert!((b.capacity_wh - 44.4).abs() < 1e-9);
        assert!((b.voltage() - Battery::default().voltage()).abs() < 1e-9);
        assert!((b.capacity_mah() - 3000.0).abs() < 1e-6);
    }

    #[test]
    fn partial_charge_lowers_voltage() {
        let full = Battery::new(4, 3000.0, 0.025, 1.0);
        let half = Battery::new(4, 3000.0, 0.025, 0.5);
        assert!(half.voltage() < full.voltage());
        assert!((half.soc() - 0.5).abs() < 1e-12);
    }
}