//! # Components
//!
//! Nominal components to label tetroid states.
//!
//! When a new tetroid is spawned, the `RigidBody` gets `ActiveTetroid`, and 
//! its colliders get `ActiveTetroidCollider`. An active tetroid keeps these
//! components until it hits the ground or any inactive tetroid/debris
//! 
//! Upon becoming inactive, the formerly `ActiveTetroid` or any new `RigidBody`(s) 
//! created by a row slice, are labelled `InactiveTetroid`--while child colliders 
//! get `InactiveTetroidCollider`
//!
//!

use bevy::ecs::component::Component;

/* Single Block plugin */
/// Nominal component used to label actively falling tetroid.
#[derive(Component)]
pub struct ActiveTetromino;

/// Tetroids are compound shapes made up of convex polygons
#[derive(Component)]
pub struct ActiveTetrominoCollider;
