//! Palette DragonSkyTriker : blancs chauds, graphite, une seule couleur d'accent (braise).

use bevy::prelude::*;

/// Fond du studio (et couleur du brouillard : l'horizon disparaît).
pub fn background() -> Color {
    Color::srgb_u8(241, 242, 246)
}
/// Sol du studio, à peine plus clair que le fond.
pub fn ground() -> Color {
    Color::srgb_u8(247, 247, 250)
}
/// Anneaux de repère au sol.
pub fn ring() -> Color {
    Color::srgb_u8(226, 227, 233)
}
/// Texte principal.
pub fn ink() -> Color {
    Color::srgb_u8(29, 29, 31)
}
/// Texte secondaire.
pub fn ink_secondary() -> Color {
    Color::srgb_u8(134, 134, 139)
}
/// Accent « braise ».
pub fn ember() -> Color {
    Color::srgb_u8(255, 107, 61)
}
/// Panneaux en verre dépoli.
pub fn glass() -> Color {
    Color::srgba(1.0, 1.0, 1.0, 0.72)
}
/// Filet très fin.
pub fn hairline() -> Color {
    Color::srgba(0.0, 0.0, 0.0, 0.07)
}
