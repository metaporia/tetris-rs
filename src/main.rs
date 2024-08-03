#![allow(unused, clippy::type_complexity, clippy::too_many_arguments)]
use std::{cmp::Ordering, collections::HashMap, f32::consts::PI};

use bevy::{prelude::*, window::WindowResolution};
use bevy_prototype_lyon::prelude::*;
use bevy_rapier2d::{geometry::RayIntersection, prelude::*};

use bevy_inspector_egui::quick::WorldInspectorPlugin;

mod arena;
mod event_demo;
pub mod image_demo;
mod setup;
mod tetroid;

use arena::{spawn_arena, Ground};
use event_demo::{get_min_max, DebugShape, RowBounds, GROUND_Y};
use tetroid::components::*;
use tetroid::{spawn_tetromino, BRICK_DIM};

// width/height of single square
const PIXELS_PER_METER: f32 = 10.0;

const IMPULSE_SCALAR: f32 = 20000.0;

fn main() {
    event_demo::app();
    //image_demo::fractal();
}

#[derive(Component)]
struct Row(u8);

#[derive(Event, Debug)]
struct Pause;

fn kbd_input(
    kbd_input: Res<ButtonInput<KeyCode>>,
    mut ext_impulses: Query<
        (
            &mut ExternalImpulse,
            &mut ExternalForce,
            &mut Velocity,
            &mut Damping,
        ),
        With<ActiveTetromino>,
    >,
    mut pause: EventWriter<Pause>,
    //active_tetroid_query: Query<Entity, With<ActiveTetroid>>,
) {
    if kbd_input.just_pressed(KeyCode::Escape) {
        pause.send(Pause);
    }

    let Ok((mut ext_impulse, mut ext_force, mut velocity, mut damping)) =
        ext_impulses.get_single_mut()
    else {
        return;
    };

    let force = 2000000.0;
    let torque = force * 15.0;
    let max_vel = 230.0;
    let max_ang_vel = 2.7;

    let mut torque_impulse = 0.0;
    let torque_imp = 13.0 * IMPULSE_SCALAR;
    if kbd_input.pressed(KeyCode::KeyZ) && velocity.angvel < max_ang_vel {
        ext_force.torque = torque;
        //velocity.angvel = -max_ang_vel;
    }
    // FIXME: enable faster max angular velocity but with less momentum (reduce
    // collider mass).
    //
    //
    // - put `ColliderMassProperties` on rigibbody
    // - remove `ColliderMassProperties` from colliders
    // - ideally we'd like them all to rotate identicially, so identical
    //   angular inertia & center of mass. Hopefully a point mass will do this.

    // - NOTE: remove all forces on `DeactivateTetroid`
    else if kbd_input.pressed(KeyCode::KeyX)
        && velocity.angvel > -max_ang_vel
    {
        ext_force.torque = -torque;
        //velocity.angvel = max_ang_vel;
    } else {
        ext_force.torque = 0.0;
    }

    let mut impulse = Vec2::new(0.0, 0.0);
    let lateral_imp = 3.0 * IMPULSE_SCALAR;
    let vertical_imp = lateral_imp;

    // cap angular and linear velocities
    // otherwise apply impulses
    if kbd_input.pressed(KeyCode::ArrowLeft) && velocity.linvel.x > -max_vel {
        ext_force.force.x = -force;
    } else if kbd_input.pressed(KeyCode::ArrowRight)
        && velocity.linvel.x < max_vel
    {
        ext_force.force.x = force;
    } else {
        ext_force.force.x = 0.0;
    }

    let max_boost_vel = -300.0;
    if kbd_input.pressed(KeyCode::ArrowDown) {
        if velocity.linvel.y > max_boost_vel {
            ext_force.force.y = -force
        }
    } else {
        ext_force.force.y = 0.0;
        // TODO: apply damping if above speed limit.
        //if velocity.linvel.y >
        if velocity.linvel.y < max_boost_vel {
            damping.linear_damping = 5.0;
        } else {
            damping.linear_damping = 0.0;
        }
        // slow down block until it reaches 250.
    }
}

// sort by lowest y, if y's are equal lowest x
fn cmp_vec2_by_y(x: &Vec2, y: &Vec2) -> Ordering {
    if x.y < y.y {
        Ordering::Less
    } else if x.y > y.y {
        Ordering::Greater
    } else if x.x < y.x {
        Ordering::Less
    } else {
        Ordering::Equal
    }
}

fn v2_to_3(Vec2 { x, y }: &Vec2) -> Vec3 {
    Vec3::new(*x, *y, 0.0)
}

fn v3_to_2(Vec3 { x, y, .. }: &Vec3) -> Vec2 {
    Vec2::new(*x, *y)
}

/// Sort so that ordered traversal yields convex polygon.
///
/// See [stackoverflow post](https://stackoverflow.com/questions/73683410/sort-vertices-of-a-convex-polygon-in-clockwise-or-counter-clockwise) for algorithm
fn sort_convex_hull(points: &mut [Vec2]) {
    // get lowest, left most point
    if !points.is_empty() {
        points.sort_by(cmp_vec2_by_y);
        let lowest = points[0];
        let atan =
            |v: &Vec2| (v.y - lowest.y).atan2(v.x - lowest.x) + 2.0 * PI;
        points.sort_by(|a, b| atan(a).partial_cmp(&atan(b)).unwrap());
    }
}

fn draw_convex_hull(mut cmds: Commands, points: Vec<Vec2>, color: Color) {
    let convex_hull = shapes::Polygon {
        points,
        closed: true,
    };
    cmds.spawn((
        ShapeBundle {
            path: GeometryBuilder::build_as(&convex_hull),
            ..Default::default()
        },
        Stroke::new(color, 2.0),
        DebugShape,
    ));
}

/// Demo to test whether ray casts that hit at origin yield correct contact
/// point (they do if `solid==true`)
fn spawn_convex_hull(mut cmds: Commands, points: &[Vec2], color: Color) {
    // NOTE: `Collider::convex_hull` panics if `points < 2`
    if points.len() >= 2 {
        if let Some(convex_hull) = Collider::convex_hull(points) {
            info!("len = {}, points = {:?}", points.len(), points);
            cmds.spawn(RigidBody::Dynamic).insert(convex_hull);
        }
    }
}

// Circle contacts
// draw small circle at each intersection
fn draw_circle_contact(center: Vec2, mut cmds: Commands) {
    let contact_point = shapes::Circle {
        radius: 2.0,
        center,
    };
    cmds.spawn((
        ShapeBundle {
            path: GeometryBuilder::build_as(&contact_point),
            ..default()
        },
        Stroke::new(Color::Srgba(Srgba::GREEN), 1.0),
    ));
}

// Draw ray
fn draw_ray(mut cmds: Commands, origin: Vec2, contact_point: Vec2) {
    let ray = shapes::Line(origin, contact_point);
    cmds.spawn((
        ShapeBundle {
            path: GeometryBuilder::build_as(&ray),
            ..default()
        },
        Stroke::new(Color::Srgba(Srgba::WHITE), 1.0),
    ));
}

mod window {
    use bevy::{prelude::*, window::WindowResolution};

    use crate::tetroid::BRICK_DIM;
    pub(super) fn plugin(app: &mut App) {
        app.add_plugins(
            DefaultPlugins
                .build()
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "Not Tetris".into(),
                        name: Some("tetris-rs".into()),
                        resize_constraints: WindowResizeConstraints {
                            min_width: BRICK_DIM * 13.0,
                            min_height: BRICK_DIM * 19.0,
                            max_width: BRICK_DIM * 15.0,
                            max_height: BRICK_DIM * 26.0,
                        },
                        fit_canvas_to_parent: false,
                        // FIXME: fix ground alignment
                        resolution: WindowResolution::new(
                            BRICK_DIM * 15.0,
                            BRICK_DIM * 25.0,
                        )
                        .with_scale_factor_override(1.0),
                        ..Default::default()
                    }),
                    ..Default::default()
                })
                //.disable::<LogPlugin>()
                //.set(LogPlugin {
                    //filter: "tetris-rs=debug,bevy_ecs=debug".to_string(),
                 //   ..Default::default()
                //}),
        );
    }
}

mod physics {
    //! Load rapier physics plugin
    use bevy::app::App;
    use bevy_rapier2d::plugin::{NoUserData, RapierPhysicsPlugin};

    use crate::PIXELS_PER_METER;

    pub(super) fn plugin(app: &mut App) {
        app.add_plugins(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(
            PIXELS_PER_METER,
        ));
    }
}

mod graphics {
    //! Load TetrominoAssetPlugin, ShapePlugin
    use bevy::app::{App, Startup};
    use bevy_prototype_lyon::plugin::ShapePlugin;

    use crate::event_demo::setup_graphics;
    use crate::image_demo::TetrominoAssetPlugin;

    pub(super) fn plugin(app: &mut App) {
        app.add_plugins(TetrominoAssetPlugin)
            .add_plugins(ShapePlugin)
            .add_systems(Startup, setup_graphics);
    }
}

