// tetroid.rs

use bevy::{prelude::*, utils::tracing::subscriber::DefaultGuard};
use bevy_rapier2d::prelude::*;

use rand::seq::SliceRandom;
use rand::Rng;

pub mod components;

use crate::{
    event_demo::{FallSpeed, Tetroid, FRICTION, GROUND_Y, INITIAL_FALLSPEED},
    image_demo::{
        new_blue_square_bundle, tetromino_type_to_sprite_bundle,
        TetrominoAssetMap,
    },
};
use components::*;

pub const BRICK_DIM: f32 = 30.0;

/// Marker type for tetromino. A `Tetromino` marks a parent `Tetroid` entity
/// with a `RigidBody` whose children are convex colliders.
#[derive(Component, Default, Debug)]
pub struct Tetromino;

/// Common components of a `Tetromino`--the parent `RigidBody`, whose children
/// are have `TetroidColliderBundle`.
#[derive(Bundle, Default)]
pub(crate) struct TetrominoBundle {
    rigid_body: RigidBody,
    sleeping: Sleeping,
    pub transform_bundle: TransformBundle,
    external_forces: ExternalForce,
    external_impulse: ExternalImpulse,
    pub velocity: Velocity,
    pub gravity_scale: GravityScale,
    tetroid: Tetroid,
    tetromino_marker: Tetromino,
    inherited_visibility: InheritedVisibility,
    damping: Damping,
}

impl TetrominoBundle {
    pub(crate) fn new(gravity_scale: f32) -> Self {
        let gravity_scale = GravityScale(gravity_scale);
        TetrominoBundle {
            gravity_scale,
            rigid_body: RigidBody::Dynamic,
            //inherited_visibility: InheritedVisibility::VISIBLE,
            ..Default::default()
        }
    }

    /// Initialize starting position with global x,y coordinates.
    pub(crate) fn with_transform<T: Into<Transform>>(
        mut self,
        transform: T,
    ) -> Self {
        self.transform_bundle =
            TransformBundle::from_transform(transform.into());
        self
    }
}

/// A `Tetromino` is (initially) made up of four square colliders. This marks
/// said colliders.
#[derive(Component, Default, Debug)]
pub struct TetrominoCollider;

/// Common components of `TetroidCollider`. Provides default `TransformBundle`,
/// collision events, and necessary marker components for a child collider of a
/// `Tetromino`
#[derive(Bundle, Default)]
pub(crate) struct TetrominoColliderBundle {
    pub tetroid: Tetroid,
    /// FIXME: test if both child and parent need this
    pub transform_bundle: TransformBundle,
    pub active_events: ActiveEvents,
    pub collider: Collider,
    pub friction: Friction,
    pub restitution: Restitution,
    pub tetroid_collider_marker: TetrominoCollider,
    pub inherited_visibility: InheritedVisibility,
}

impl TetrominoColliderBundle {
    /// Creates `TetroidColliderBundle` from `Collider`.
    ///
    /// It is assumed that `Collider` is a `Collider::convex_hull`.
    pub(crate) fn new(collider: Collider, friction_coefficient: f32) -> Self {
        TetrominoColliderBundle {
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
            ..Default::default()
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

    /// Initialize starting position with global x,y coordinates.
    pub(crate) fn with_transform(mut self, transform: Transform) -> Self {
        self.transform_bundle = TransformBundle::from_transform(transform);
        self
    }
}

#[derive(Debug, Clone, Copy, Eq, Hash, PartialEq)]
pub enum TetrominoType {
    // symmetric
    I,
    O,
    T,
    // asymmetric
    J, // reflection of L
    L,
    S,
    Z, // reflection of S
}

impl TetrominoType {
    pub fn as_idx(&self) -> usize {
        match self {
            Self::I => 1,
            Self::J => 2,
            Self::L => 3,
            Self::O => 4,
            Self::S => 5,
            Self::T => 6,
            Self::Z => 7,
        }
    }

    pub fn from_idx(i: usize) -> Option<Self> {
        match i {
            1 => Some(Self::I),
            2 => Some(Self::J),
            3 => Some(Self::L),
            4 => Some(Self::O),
            5 => Some(Self::S),
            6 => Some(Self::T),
            7 => Some(Self::Z),
            _ => None,
        }
    }

    /// Pick a random `Tetromino` variant.
    pub fn random() -> Option<Self> {
        let mut rng = rand::thread_rng();
        Self::VARIANTS.choose(&mut rng).copied().map(|t| {
            dbg!(t);
            t
        })
    }

    /// Pick a random variant and convert it into a `Vec` of relative position
    /// and collider pairs.
    pub fn random_colliders() -> Option<Vec<(Vec2, Collider)>> {
        Self::random().map(Self::to_colliders)
    }

    pub fn to_colliders(self) -> Vec<(Vec2, Collider)> {
        match self {
            Self::I => Self::with_squares(&Self::IBLOCK),
            Self::O => Self::with_squares(&Self::OBLOCK),
            Self::L => Self::with_squares(&Self::LBLOCK),
            Self::J => Self::with_squares(&Self::reflect(&Self::LBLOCK)),
            Self::Z => Self::with_squares(&Self::ZBLOCK),
            Self::S => Self::with_squares(&Self::reflect(&Self::ZBLOCK)),
            Self::T => Self::with_squares(&Self::TBLOCK),
        }
    }

    pub fn square() -> Collider {
        Collider::convex_hull(&Self::SQUARE).unwrap()
    }

    fn with_squares(positions: &[Vec2]) -> Vec<(Vec2, Collider)> {
        positions
            .iter()
            .copied()
            .map(|p| (p, Self::square()))
            .collect()
    }

    /// Reflect a slice of relative positions about the x-axis.
    fn reflect(positions: &[Vec2]) -> Vec<Vec2> {
        positions
            .iter()
            .copied()
            .map(|Vec2 { x, y }| Vec2::new(x * -1.0, y))
            .collect()
    }

    const VARIANTS: [Self; 7] = [
        Self::I,
        Self::O,
        Self::L,
        Self::Z,
        Self::J,
        Self::T,
        Self::S,
    ];

    pub const SQUARE: [Vec2; 4] = [
        Vec2::new(0.0, 0.0),
        Vec2::new(0.0, BRICK_DIM),
        Vec2::new(BRICK_DIM, 0.0),
        Vec2::new(BRICK_DIM, BRICK_DIM),
    ];

    const IBLOCK: [Vec2; 4] = [
        Vec2::new(0.0, 0.0),
        Vec2::new(0.0, -BRICK_DIM),
        Vec2::new(0.0, -2.0 * BRICK_DIM),
        Vec2::new(0.0, -3.0 * BRICK_DIM),
    ];

    const OBLOCK: [Vec2; 4] = [
        Vec2::new(0.0, 0.0),
        Vec2::new(-BRICK_DIM, -BRICK_DIM),
        Vec2::new(-BRICK_DIM, 0.0),
        Vec2::new(0.0, -BRICK_DIM),
    ];

    const ZBLOCK: [Vec2; 4] = [
        Vec2::new(0.0, 0.0),
        Vec2::new(0.0, -BRICK_DIM),
        Vec2::new(-BRICK_DIM, -BRICK_DIM),
        Vec2::new(-BRICK_DIM, 2.0 * -BRICK_DIM),
    ];

    const TBLOCK: [Vec2; 4] = [
        Vec2::new(0.0, 0.0),
        Vec2::new(-BRICK_DIM, 0.0),
        Vec2::new(-BRICK_DIM * 2.0, 0.0),
        Vec2::new(-BRICK_DIM, -BRICK_DIM),
    ];

    const LBLOCK: [Vec2; 4] = [
        // lblock has four squares
        // | 0 |
        // | 1 |
        // | 2 | 3 |
        // NB. coordinates are in local space, squares have no rotation
        Vec2::new(0.0, 0.0),
        Vec2::new(0.0, -BRICK_DIM),
        Vec2::new(0.0, -2.0 * BRICK_DIM),
        Vec2::new(BRICK_DIM, -2.0 * BRICK_DIM),
    ];
}

pub fn spawn_tetromino(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    fallspeed: Res<FallSpeed>,
    asset_map: Res<TetrominoAssetMap>,
) {
    //let square = Collider::cuboid(BRICK_DIM / 2.0, BRICK_DIM / 2.0);
    let square = TetrominoType::square();

    let tetromino_type = TetrominoType::random().unwrap();
    let tetromino = tetromino_type.to_colliders();

    // active tetroid gets lower gravity
    let mut tetromino_bundle = TetrominoBundle {
        gravity_scale: GravityScale(0.0),
        velocity: fallspeed.as_velocity(),
        transform_bundle: TransformBundle::from(Transform::from_xyz(
            -BRICK_DIM,
            GROUND_Y + BRICK_DIM * 21.0,
            0.0,
        )),
        ..Default::default()
    };

    let id2 = commands
        .spawn(tetromino_bundle)
        .insert(ActiveTetromino)
        .with_children(|children| {
            tetromino.into_iter().for_each(|(Vec2 { x, y }, collider)| {
                let collider_bundle = TetrominoColliderBundle {
                    collider,
                    ..Default::default()
                }
                .with_friction(FRICTION)
                .with_starting_position(x, y);

                let block_sprite = tetromino_type_to_sprite_bundle(
                    &tetromino_type,
                    images.as_mut(),
                    asset_map.as_ref(),
                );
                let blue_square_bundle =
                    new_blue_square_bundle(images.as_mut());
                children
                    .spawn(collider_bundle)
                    .insert(ActiveTetrominoCollider)
                    .with_children(|cs| {
                        cs.spawn(block_sprite);
                    });
            })
        })
        .id();

    info!("Spawned new lblock: {:?}", id2);
}
