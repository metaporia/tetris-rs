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
pub struct ActiveTetroid;

/// Tetroids are compound shapes made up of convex polygons
#[derive(Component)]
pub struct ActiveTetroidCollider;


/// Indicates an inactive `RigidBody` in the tetroid debris field.
/// - May be concave
/// - Children are `DerbrisCollider`s
#[derive(Component)]
pub struct Debris;

/// Colliders attached to inactive `RigidBody`s in tetroid debris field.
#[derive(Component)]
pub struct DebrisCollider;
