#![allow(unused, clippy::type_complexity, clippy::too_many_arguments)]
use std::{cmp::Ordering, collections::HashMap, f32::consts::PI};

use bevy::{prelude::*, window::WindowResolution};
use bevy_prototype_lyon::prelude::*;
use bevy_rapier2d::{
    geometry::RayIntersection,
    prelude::*,
};

use bevy_inspector_egui::quick::WorldInspectorPlugin;

mod arena;
mod event_demo;
mod tetroid;

use arena::{spawn_arena, Ground};
use event_demo::DebugShape;
use tetroid::components::*;
use tetroid::{spawn_lblock, BRICK_DIM};

// width/height of single square
const PIXELS_PER_METER: f32 = 50.0;

const IMPULSE_SCALAR: f32 = 20000.0;

fn main() {
    event_demo::app();
}

#[derive(Event)]
struct HitGround(Entity);

#[derive(Event)]
struct NextTetroid;

#[derive(Component)]
struct Row(u8);

#[derive(Component, Debug)]
struct Vertices {
    vs: Vec<Vec2>,
}

#[derive(Event, Debug)]
struct Pause;

fn kbd_input(
    kbd_input: Res<ButtonInput<KeyCode>>,
    mut ext_impulses: Query<&mut ExternalImpulse, With<ActiveTetroid>>,
    mut pause: EventWriter<Pause>,
    //active_tetroid_query: Query<Entity, With<ActiveTetroid>>,
) {
    if kbd_input.just_pressed(KeyCode::Escape) {
        pause.send(Pause);
    }

    let mut torque_impulse = 0.0;
    let torque_imp = 10.0 * IMPULSE_SCALAR;
    if kbd_input.pressed(KeyCode::KeyZ) {
        torque_impulse = torque_imp;
    } else if kbd_input.pressed(KeyCode::KeyX) {
        torque_impulse = -torque_imp;
    }

    let mut impulse = Vec2::new(0.0, 0.0);
    let lateral_imp = 5.0 * IMPULSE_SCALAR;
    let vertical_imp = lateral_imp;
    
    if kbd_input.pressed(KeyCode::ArrowLeft) {
        impulse.x = -lateral_imp;
    }
    if kbd_input.pressed(KeyCode::ArrowRight) {
        impulse.x = lateral_imp;
    }
    if kbd_input.pressed(KeyCode::ArrowDown) {
        impulse.y = -vertical_imp;
    }
    for mut ext_impulse in ext_impulses.iter_mut() {
        //println!(
        //    "impulse: {}, lateral_impulse: {}",
        //    &impulse, &lateral_impulse
        //);
        //println!(
        //    "torque: {}, lateral: {}",
        //    &ext_impulse.torque_impulse, &ext_impulse.impulse
        //);
        *ext_impulse = ExternalImpulse {
            torque_impulse,
            impulse,
        };
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
        DebugShape
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
        Stroke::new(Color::Srgba(Srgba::BLACK), 1.0),
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
