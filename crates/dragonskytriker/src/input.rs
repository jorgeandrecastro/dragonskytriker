//! Touches logiques : insensibles à la disposition (AZERTY/QWERTY) et à la casse.

use bevy::input::keyboard::Key;
use bevy::prelude::*;

fn is_char(k: &Key, c: &str) -> bool {
    matches!(k, Key::Character(s) if s.eq_ignore_ascii_case(c))
}

/// Vrai si la touche caractère `c` vient d'être pressée.
pub fn just(keys: &ButtonInput<Key>, c: &str) -> bool {
    keys.get_just_pressed().any(|k| is_char(k, c))
}

/// Vrai si l'une des touches `cs` est maintenue.
pub fn held(keys: &ButtonInput<Key>, cs: &[&str]) -> bool {
    keys.get_pressed().any(|k| cs.iter().any(|c| is_char(k, c)))
}