// tetroid.rs

use bevy::{prelude::*, utils::tracing::subscriber::DefaultGuard};
use bevy_rapier2d::prelude::*;

pub mod components;

use crate::event_demo::{Tetroid, GROUND_Y};
use components::*;

pub const BRICK_DIM: f32 = 30.0;

/// Common components of a `Tetromino`--the parent `RigidBody`, whose children
/// are have `TetroidColliderBundle`.
#[derive(Bundle, Default)]
pub(crate) struct TetrominoBundle {
    rigid_body: RigidBody,
    sleeping: Sleeping,
    pub transform_bundle: TransformBundle,
    external_impulse: ExternalImpulse,
    pub velocity: Velocity,
    pub gravity_scale: GravityScale,
    tetroid: Tetroid,
}

impl TetrominoBundle {
    pub(crate) fn new(gravity_scale: f32) -> Self {
        let gravity_scale = GravityScale(gravity_scale);
        TetrominoBundle {
            gravity_scale,
            rigid_body: RigidBody::Dynamic,
            ..Default::default()
        }
    }
}

/// Common components of `TetroidCollider`. Provides default `TransformBundle`,
/// collision events, and necessary marker components for a child collider of a
/// `Tetromino`
#[derive(Bundle, Default)]
pub(crate) struct TetroidColliderBundle {
    tetroid: Tetroid,
    /// FIXME: test if both child and parent need this
    pub transform_bundle: TransformBundle,
    active_events: ActiveEvents,
    collider: Collider,
    friction: Friction,
    restitution: Restitution,
}

impl TetroidColliderBundle {
    /// Creates `TetroidColliderBundle` from `Collider`.
    ///
    /// It is assumed that `Collider` is a `Collider::convex_hull`.
    pub(crate) fn new(collider: Collider, friction_coefficient: f32) -> Self {
        TetroidColliderBundle {
            collider,
            tetroid: Tetroid,
            transform_bundle: TransformBundle::IDENTITY,
            active_events: ActiveEvents::COLLISION_EVENTS,
            friction: Friction {
                coefficient: friction_coefficient,
                combine_rule: CoefficientCombineRule::Min,
            },
            restitution: Restitution {
                coefficient: 0.0,
                combine_rule: CoefficientCombineRule::Min,
            },
        }
    }

    // TODO: use builder pattern. These duplicate initialization of several
    // components.
    //
    /// Add friction
    pub(crate) fn with_friction(mut self, friction_coefficient: f32) -> Self {
        self.friction.coefficient = friction_coefficient;
        self
    }

    /// Initialize starting position with global x,y coordinates.
    pub(crate) fn with_starting_position(mut self, x: f32, y: f32) -> Self {
        self.transform_bundle.local = Transform::from_xyz(x, y, 0.0);
        self
    }
}

pub fn spawn_lblock(mut commands: Commands) {
    //let square = Collider::cuboid(BRICK_DIM / 2.0, BRICK_DIM / 2.0);
    let square = Collider::convex_hull(&[
        Vec2::new(0.0, 0.0),
        Vec2::new(0.0, BRICK_DIM),
        Vec2::new(BRICK_DIM, 0.0),
        Vec2::new(BRICK_DIM, BRICK_DIM),
    ])
    .unwrap();

    let lblock_compound = Collider::compound(vec![
        // lblock has four squares
        // | 0 |
        // | 1 |
        // | 2 | 3 |
        // NB. coordinates are in local space, squares have no rotation
        // NOTE: These must be ordered so that each element is contiguous with
        // the elements before and after it.
        (Vec2::new(0.0, 0.0), 0.0, square.clone()),
        (Vec2::new(0.0, -BRICK_DIM), 0.0, square.clone()),
        (Vec2::new(0.0, -2.0 * BRICK_DIM), 0.0, square.clone()),
        (Vec2::new(BRICK_DIM, -2.0 * BRICK_DIM), 0.0, square.clone()),
    ]);

    let lblock_components = [
        // lblock has four squares
        // | 0 |
        // | 1 |
        // | 2 | 3 |
        // NB. coordinates are in local space, squares have no rotation
        (Vec2::new(0.0, 0.0), square.clone()),
        (Vec2::new(0.0, -BRICK_DIM), square.clone()),
        (Vec2::new(0.0, -2.0 * BRICK_DIM), square.clone()),
        (Vec2::new(BRICK_DIM, -2.0 * BRICK_DIM), square.clone()),
    ];

    // active tetroid gets lower gravity
    let mut tetromino_bundle = TetrominoBundle {
        gravity_scale: GravityScale(0.2),
        velocity: Velocity {
            linvel: Vec2::new(0.0, -120.0),
            angvel: 0.0,
        },
        transform_bundle: TransformBundle::from(Transform::from_xyz(
            -BRICK_DIM,
            GROUND_Y + BRICK_DIM * 21.0,
            0.0,
        )),
        ..Default::default()
    };

    let id2 = commands
        .spawn(tetromino_bundle)
        .insert(ActiveTetroid)
        .with_children(|children| {
            lblock_components.clone().into_iter().for_each(
                |(Vec2 { x, y }, collider)| {
                    let mut collider_bundle = TetroidColliderBundle {
                        collider,
                        ..Default::default()
                    }
                    .with_friction(FRICTION)
                    .with_starting_position(x, y);
                    children
                        .spawn(collider_bundle)
                        .insert(ActiveTetroidCollider);
                },
            )
        })
        .id();

    //let id = commands
    //    .spawn(RigidBody::Dynamic)
    //    .insert(Sleeping::default())
    //    .with_children(|children| {
    //        lblock_components.into_iter().for_each(
    //            |(Vec2 { x, y }, shape)| {
    //                children
    //                    .spawn(shape)
    //                    .insert(ActiveTetroidCollider)
    //                    .insert(Tetroid)
    //                    .insert(TransformBundle::from(Transform::from_xyz(
    //                        x, y, 0.0,
    //                    )))
    //                    .insert(ActiveEvents::COLLISION_EVENTS)
    //                    .insert(Friction {
    //                        coefficient: 0.0,
    //                        combine_rule: CoefficientCombineRule::Min,
    //                    })
    //                    .insert(Restitution {
    //                        coefficient: 0.0,
    //                        combine_rule: CoefficientCombineRule::Min,
    //                    });
    //            },
    //        );
    //    })
    //    .insert(TransformBundle::from(Transform::from_xyz(
    //        -BRICK_DIM,
    //        GROUND_Y + BRICK_DIM * 21.0,
    //        0.0,
    //    )))
    //    .insert(ExternalImpulse {
    //        impulse: Vec2::new(0.0, 0.0),
    //        torque_impulse: 0.0,
    //    })
    //    //.insert(ActiveEvents::COLLISION_EVENTS)
    //    .insert(Ccd::enabled()) // enable continous collision detection
    //    .insert(ActiveTetroid)
    //    .insert(Tetroid)
    //    .insert(Velocity {
    //        linvel: Vec2::new(0.0, -120.0),
    //        angvel: 0.0,
    //    })
    //    .insert(GravityScale(0.02))
    //    .id();
    commands.entity(id2).log_components();
    info!("Spawned new lblock: {:?}", id2);
}
