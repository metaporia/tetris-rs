// tetroid.rs

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

pub mod components;

use crate::event_demo::{Tetroid, GROUND_Y};
use components::*;

pub const BRICK_DIM: f32 = 30.0;

#[derive(Component)]
pub(crate) struct ContiguityIndex(usize);

impl ContiguityIndex {
    pub(crate) fn contiguous_with(&self, other: &Self) -> bool {
        if self.0 == 0 {
            other.0 == 1
        } else {
            self.0 - 1 == other.0 || self.0 + 1 == other.0
        }
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

    let id = commands
        .spawn(RigidBody::Dynamic)
        .insert(Sleeping::default())
        .with_children(|children| {
            lblock_components.into_iter().enumerate().for_each(
                |(i, (Vec2 { x, y }, shape))| {
                    children
                        .spawn(shape)
                        //.insert(ContiguityIndex(i))
                        .insert(ActiveTetroidCollider)
                        .insert(Tetroid)
                        .insert(TransformBundle::from(Transform::from_xyz( x, y, 0.0,)))
                        .insert(ActiveEvents::COLLISION_EVENTS)
                        .insert(Friction {
                            coefficient: 0.0,
                            combine_rule: CoefficientCombineRule::Min,
                        })
                        .insert(Restitution {
                            coefficient: 0.0,
                            combine_rule: CoefficientCombineRule::Min,
                        });
                },
            );
        })
        .insert(TransformBundle::from(Transform::from_xyz(
            -BRICK_DIM,
            GROUND_Y + BRICK_DIM * 21.0,
            0.0,
        )))
        .insert(ExternalImpulse {
            impulse: Vec2::new(0.0, 0.0),
            torque_impulse: 0.0,
        })
        //.insert(ActiveEvents::COLLISION_EVENTS)
        .insert(Ccd::enabled()) // enable continous collision detection
        .insert(ActiveTetroid)
        .insert(Tetroid)
        .insert(Velocity {
            linvel: Vec2::new(0.0, -120.0),
            angvel: 0.0,
        })
        .insert(GravityScale(0.02))
        .id();
    //commands.entity(id).log_components();
    info!("Spawned new lblock: {:?}", id);
}
