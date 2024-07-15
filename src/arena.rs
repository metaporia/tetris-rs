// arena.rs

use bevy::prelude::*;
use bevy_prototype_lyon::{
    draw::{Fill, Stroke},
    entity::ShapeBundle,
    geometry::GeometryBuilder,
    shapes,
};
use bevy_rapier2d::prelude::*;

use crate::tetroid::BRICK_DIM;

const OUTLINE_THICKNESS: f32 = 1.0;

#[derive(Component)]
pub struct Ground;

#[derive(Component, Debug)]
pub enum Wall {
    Left,
    Right,
}

pub fn spawn_arena(mut commands: Commands) {
    let brick_half: f32 = BRICK_DIM / 2.0;

    let ground_extents = Vect {
        x: BRICK_DIM * 5.0,
        y: BRICK_DIM / 2.0,
    };
    let ground_origin_y = -BRICK_DIM * (18.0 / 2.0) - 0.5 * BRICK_DIM;
    let ground_shape = shapes::Rectangle {
        extents: Vect {
            x: ground_extents.x * 2.0 - OUTLINE_THICKNESS,
            y: ground_extents.y * 2.0 - OUTLINE_THICKNESS,
        },
        ..Default::default()
    };
    /* Create the ground. */
    commands
        //.spawn((
        //    ShapeBundle {
        //        path: bevy_prototype_lyon::geometry::GeometryBuilder::build_as(
        //            &ground_shape,
        //        ),
        //        ..default()
        //    },
        //    Fill::color(Color::BLACK),
        //    Stroke::new(Color::BLACK, OUTLINE_THICKNESS),
        //))
        .spawn(RigidBody::Fixed)
        .insert(ColliderMassProperties::Mass(100.0))
        .insert(Collider::cuboid(ground_extents.x, ground_extents.y))
        .insert(Restitution::coefficient(0.0))
        .insert(Friction::new(0.0))
        .insert(TransformBundle::from(Transform::from_xyz(
            0.0,
            ground_origin_y,
            0.0,
        )))
        .insert(Ground);

    /* Create left wall */
    let wall_shape = shapes::Rectangle {
        extents: Vect {
            x: BRICK_DIM - OUTLINE_THICKNESS,
            y: BRICK_DIM * 19.0 - OUTLINE_THICKNESS,
        },
        ..Default::default()
    };
    commands
        //.spawn((
        //    ShapeBundle {
        //        path: GeometryBuilder::build_as(&wall_shape),
        //        ..default()
        //    },
        //    Fill::color(Color::BLACK),
        //    Stroke::new(Color::BLACK, OUTLINE_THICKNESS),
        //))
        .spawn(Wall::Left)
        .insert(Collider::cuboid(brick_half, BRICK_DIM * 9.5))
        .insert(Friction::new(0.0))
        .insert(Restitution::coefficient(0.0))
        .insert(TransformBundle::from(Transform::from_xyz(
            -BRICK_DIM * 5.0 - brick_half,
            -BRICK_DIM * 0.5,
            0.0,
        )));

    // right wall
    commands
        //.spawn((
        //    ShapeBundle {
        //        path: GeometryBuilder::build_as(&wall_shape),
        //        ..default()
        //    },
        //    Fill::color(Color::BLACK),
        //    Stroke::new(Color::BLACK, OUTLINE_THICKNESS),
        //))
        .spawn(Collider::cuboid(brick_half, BRICK_DIM * 9.5))
        .insert(Restitution::coefficient(0.0))
        .insert(Friction::new(0.0))
        .insert(Wall::Right)
        .insert(TransformBundle::from(Transform::from_xyz(
            BRICK_DIM * 5.0 + brick_half,
            -BRICK_DIM * 0.5,
            0.0,
        )));
}
