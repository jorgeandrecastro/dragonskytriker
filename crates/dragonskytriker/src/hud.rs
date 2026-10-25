//! HUD épuré : verre dépoli, typographie Inter, une seule couleur d'accent.

use bevy::input::keyboard::Key;
use bevy::prelude::*;

use crate::camera::CameraRig;
use crate::input::just;
use crate::sim::Flight;
use crate::theme;
use dragonskytriker_core::FlightMode;

/// Interface : nom, mode de vol, télémétrie, moteurs, aide clavier.
pub struct HudPlugin;

impl Plugin for HudPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, (spawn_hud, spawn_info_card))
            .add_systems(Update, (update_readouts, update_motor_bars, toggle_hud, toggle_info));
    }
}

#[derive(Component)]
struct HudRoot;

#[derive(Component, Clone, Copy)]
enum Readout {
    Mode,
    Camera,
    Nav,
    Altitude,
    Speed,
    Heading,
    Tilt,
    Battery,
    Power,
    Wind,
    Baro,
    Gnss,
    Lidar,
    Imu,
    Estimate,
}

#[derive(Component)]
struct InfoCard;

#[derive(Component)]
struct MotorBar(usize);

#[derive(Component)]
struct ModeDot;

fn font(assets: &AssetServer, size: f32) -> TextFont {
    TextFont {
        font: assets.load("fonts/Inter.ttf").into(),
        font_size: size.into(),
        ..default()
    }
}

fn spawn_hud(mut commands: Commands, assets: Res<AssetServer>) {
    let glass_node = |px: f32, py: f32| Node {
        flex_direction: FlexDirection::Row,
        align_items: AlignItems::Center,
        padding: UiRect::axes(Val::Px(px), Val::Px(py)),
        border: UiRect::all(Val::Px(1.0)),
        border_radius: BorderRadius::all(Val::Px(22.0)),
        ..default()
    };

    commands
        .spawn((
            HudRoot,
            Node {
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                position_type: PositionType::Absolute,
                flex_direction: FlexDirection::Column,
                justify_content: JustifyContent::SpaceBetween,
                padding: UiRect::all(Val::Px(30.0)),
                ..default()
            },
        ))
        .with_children(|root| {
            // ---- Ligne du haut ----
            root.spawn(Node {
                flex_direction: FlexDirection::Row,
                justify_content: JustifyContent::SpaceBetween,
                align_items: AlignItems::FlexStart,
                ..default()
            })
            .with_children(|row| {
                row.spawn(Node { flex_direction: FlexDirection::Column, row_gap: Val::Px(4.0), ..default() })
                    .with_children(|c| {
                        c.spawn((Text::new("DragonSkyTriker"), font(&assets, 22.0), TextColor(theme::ink())));
                        c.spawn((
                            Text::new("Dragon X1  ·  Simulation"),
                            font(&assets, 12.0),
                            TextColor(theme::ink_secondary()),
                        ));
                    });

                row.spawn(Node { flex_direction: FlexDirection::Row, column_gap: Val::Px(10.0), ..default() })
                    .with_children(|pills| {
                        pills
                            .spawn((glass_node(16.0, 9.0), BackgroundColor(theme::glass()), BorderColor::all(theme::hairline())))
                            .with_children(|p| {
                                p.spawn((Text::new("Camera"), font(&assets, 12.0), TextColor(theme::ink_secondary())));
                                p.spawn((
                                    Node { margin: UiRect::left(Val::Px(8.0)), ..default() },
                                    Text::new("Chase"),
                                    font(&assets, 13.0),
                                    TextColor(theme::ink()),
                                    Readout::Camera,
                                ));
                            });
                        pills
                            .spawn((glass_node(16.0, 9.0), BackgroundColor(theme::glass()), BorderColor::all(theme::hairline())))
                            .with_children(|p| {
                                p.spawn((Text::new("Nav"), font(&assets, 12.0), TextColor(theme::ink_secondary())));
                                p.spawn((
                                    Node { margin: UiRect::left(Val::Px(8.0)), ..default() },
                                    Text::new("Truth"),
                                    font(&assets, 13.0),
                                    TextColor(theme::ink()),
                                    Readout::Nav,
                                ));
                            });
                        pills
                            .spawn((glass_node(16.0, 9.0), BackgroundColor(theme::glass()), BorderColor::all(theme::hairline())))
                            .with_children(|p| {
                                p.spawn((
                                    ModeDot,
                                    Node {
                                        width: Val::Px(8.0),
                                        height: Val::Px(8.0),
                                        margin: UiRect::right(Val::Px(9.0)),
                                        border_radius: BorderRadius::all(Val::Px(4.0)),
                                        ..default()
                                    },
                                    BackgroundColor(theme::ink_secondary()),
                                ));
                                p.spawn((Text::new("Disarmed"), font(&assets, 13.0), TextColor(theme::ink()), Readout::Mode));
                            });
                    });
            });

            // ---- Ligne du bas ----
            root.spawn(Node {
                flex_direction: FlexDirection::Row,
                justify_content: JustifyContent::SpaceBetween,
                align_items: AlignItems::FlexEnd,
                ..default()
            })
            .with_children(|row| {
                // Aide clavier.
                row.spawn(Node { width: Val::Px(270.0), flex_direction: FlexDirection::Column, row_gap: Val::Px(5.0), ..default() })
                    .with_children(|c| {
                        for line in [
                            "T  Takeoff     L  Land     H  Hold",
                            "M  Mission     R  Reset    C  Camera",
                            "WASD / ZQSD  Move     ← →  Yaw",
                            "Space / ↑  Climb     ↓  Descend",
                            "E  Fly on estimate     G  Wind     V  Sensors",
                            "F  Record     I  Info     Tab  Hide interface",
                        ] {
                            c.spawn((Text::new(line), font(&assets, 11.0), TextColor(theme::ink_secondary())));
                        }
                    });

                // Télémétrie.
                row.spawn((glass_node(28.0, 16.0), BackgroundColor(theme::glass()), BorderColor::all(theme::hairline())))
                    .with_children(|panel| {
                        for (label, which, init) in [
                            ("ALTITUDE", Readout::Altitude, "0.00 m"),
                            ("SPEED", Readout::Speed, "0.0 m/s"),
                            ("HEADING", Readout::Heading, "000°"),
                            ("TILT", Readout::Tilt, "0.0°"),
                            ("BATTERY", Readout::Battery, "100 %"),
                        ] {
                            panel
                                .spawn(Node {
                                    flex_direction: FlexDirection::Column,
                                    row_gap: Val::Px(3.0),
                                    min_width: Val::Px(82.0),
                                    ..default()
                                })
                                .with_children(|c| {
                                    c.spawn((Text::new(label), font(&assets, 10.0), TextColor(theme::ink_secondary())));
                                    c.spawn((Text::new(init), font(&assets, 20.0), TextColor(theme::ink()), which));
                                });
                        }
                    });

                // Moteurs.
                row.spawn(Node { width: Val::Px(270.0), justify_content: JustifyContent::FlexEnd, ..default() })
                    .with_children(|c| {
                        c.spawn((glass_node(18.0, 14.0), BackgroundColor(theme::glass()), BorderColor::all(theme::hairline())))
                            .with_children(|panel| {
                                panel
                                    .spawn(Node {
                                        height: Val::Px(34.0),
                                        column_gap: Val::Px(7.0),
                                        align_items: AlignItems::FlexEnd,
                                        ..default()
                                    })
                                    .with_children(|bars| {
                                        for i in 0..4 {
                                            bars.spawn((
                                                MotorBar(i),
                                                Node {
                                                    width: Val::Px(7.0),
                                                    height: Val::Px(4.0),
                                                    border_radius: BorderRadius::all(Val::Px(3.5)),
                                                    ..default()
                                                },
                                                BackgroundColor(theme::ember()),
                                            ));
                                        }
                                    });
                            });
                    });
            });
        });
}

fn spawn_info_card(mut commands: Commands, assets: Res<AssetServer>) {
    commands
        .spawn((
            InfoCard,
            Node {
                position_type: PositionType::Absolute,
                right: Val::Px(30.0),
                top: Val::Px(92.0),
                width: Val::Px(262.0),
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(9.0),
                padding: UiRect::axes(Val::Px(18.0), Val::Px(16.0)),
                border: UiRect::all(Val::Px(1.0)),
                border_radius: BorderRadius::all(Val::Px(20.0)),
                ..default()
            },
            BackgroundColor(theme::glass()),
            BorderColor::all(theme::hairline()),
        ))
        .with_children(|card| {
            card.spawn((Text::new("SENSORS"), font(&assets, 10.0), TextColor(theme::ink_secondary())));
            for (label, which, init) in [
                ("IMU", Readout::Imu, "9.81 m/s²"),
                ("Barometer", Readout::Baro, "0.00 m"),
                ("GNSS", Readout::Gnss, "—"),
                ("Lidar", Readout::Lidar, "—"),
                ("Wind", Readout::Wind, "0.0 m/s"),
                ("Power", Readout::Power, "0 W"),
                ("Estimate error", Readout::Estimate, "—"),
            ] {
                card.spawn(Node {
                    flex_direction: FlexDirection::Row,
                    justify_content: JustifyContent::SpaceBetween,
                    align_items: AlignItems::Center,
                    ..default()
                })
                .with_children(|row| {
                    row.spawn((Text::new(label), font(&assets, 11.0), TextColor(theme::ink_secondary())));
                    row.spawn((Text::new(init), font(&assets, 12.0), TextColor(theme::ink()), which));
                });
            }
        });
}

fn mode_label(m: FlightMode) -> &'static str {
    match m {
        FlightMode::Disarmed => "Disarmed",
        FlightMode::Takeoff => "Takeoff",
        FlightMode::Hover => "Hover",
        FlightMode::Mission => "Mission",
        FlightMode::Land => "Landing",
        FlightMode::Manual => "Manual",
    }
}

fn update_readouts(
    flight: Res<Flight>,
    rig: Res<CameraRig>,
    mut texts: Query<(&Readout, &mut Text)>,
    mut dot: Query<&mut BackgroundColor, With<ModeDot>>,
) {
    let s = &flight.quad.state;
    let mode = flight.autopilot.mode();
    let latest = &flight.sensors.latest;
    for (r, mut text) in &mut texts {
        text.0 = match r {
            Readout::Mode => if s.crashed { "Crashed".to_string() } else { mode_label(mode).to_string() },
            Readout::Camera => rig.mode.label().to_string(),
            Readout::Nav => if flight.use_estimator { "Estimate" } else { "Truth" }.to_string(),
            Readout::Altitude => format!("{:.2} m", s.altitude()),
            Readout::Speed => format!("{:.1} m/s", s.speed()),
            Readout::Heading => {
                let deg = (-s.yaw().to_degrees()).rem_euclid(360.0);
                format!("{:03.0}°", deg)
            }
            Readout::Tilt => format!("{:.1}°", s.tilt().to_degrees()),
            Readout::Battery => format!("{:.0} %", s.battery.soc() * 100.0),
            Readout::Power => format!("{:.0} W  ·  {:.1} V", s.power, s.battery.voltage()),
            Readout::Wind => {
                let w = flight.env.wind.velocity();
                let speed = (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]).sqrt();
                let bearing = w[0].atan2(-w[2]).to_degrees().rem_euclid(360.0);
                format!("{:.1} m/s  ·  {:03.0}°", speed, bearing)
            }
            Readout::Baro => format!("{:.2} m  ·  {:.1} kPa", latest.baro.altitude, latest.baro.pressure / 1000.0),
            Readout::Gnss => match latest.gnss {
                Some(f) => format!("{:.5}, {:.5}", f.latitude, f.longitude),
                None => "searching".to_string(),
            },
            Readout::Lidar => match latest.lidar.nearest() {
                Some(d) => format!("nearest {:.2} m", d),
                None => "clear".to_string(),
            },
            Readout::Imu => {
                let a = latest.imu.accel;
                let g = latest.imu.gyro;
                let an = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
                let gn = (g[0] * g[0] + g[1] * g[1] + g[2] * g[2]).sqrt();
                format!("{:.2} m/s²  ·  {:.0} °/s", an, gn.to_degrees())
            }
            Readout::Estimate => format!(
                "{:.1}°  ·  {:.2} m",
                flight.estimator.attitude_error(s).to_degrees(),
                flight.estimator.position_error(s)
            ),
        };
    }
    if let Ok(mut bg) = dot.single_mut() {
        bg.0 = if mode == FlightMode::Disarmed && !s.crashed { theme::ink_secondary() } else { theme::ember() };
    }
}

fn update_motor_bars(flight: Res<Flight>, mut bars: Query<(&MotorBar, &mut Node)>) {
    let max = flight.quad.params.max_motor_thrust;
    for (bar, mut node) in &mut bars {
        let load = (flight.quad.state.motors[bar.0] / max).clamp(0.0, 1.0) as f32;
        node.height = Val::Px(4.0 + 30.0 * load);
    }
}

fn toggle_hud(
    keys: Res<ButtonInput<KeyCode>>,
    mut root: Query<&mut Visibility, (With<HudRoot>, Without<InfoCard>)>,
    mut card: Query<&mut Visibility, (With<InfoCard>, Without<HudRoot>)>,
) {
    if keys.just_pressed(KeyCode::Tab) {
        let hide = root.iter().next().map(|v| *v != Visibility::Hidden).unwrap_or(false);
        let target = if hide { Visibility::Hidden } else { Visibility::Inherited };
        for mut v in &mut root {
            *v = target;
        }
        for mut v in &mut card {
            *v = target;
        }
    }
}

fn toggle_info(keys: Res<ButtonInput<Key>>, mut card: Query<&mut Visibility, With<InfoCard>>) {
    if just(&keys, "i") {
        for mut v in &mut card {
            *v = if *v == Visibility::Hidden { Visibility::Inherited } else { Visibility::Hidden };
        }
    }
}