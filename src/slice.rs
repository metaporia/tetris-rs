//! Handles ray casting (see `row_intersections`), partitioning (see
//! `partitions`), and exposes a system set for the row density plugin to
//! hook into.

// FIXME::
// - image slices are using an old transform.
// - ok so the damping /does/ make it slightly worse. and it probaly has to
//   due with transform updates. But in order to make it perfect we need to
//   allow the colliders to rebound before freezing and slicing.
//   we could add a state before Frozen, ie, Freezing, in which we remove
//   all velocities but allow contact forces to remain for like two frames,
//   and then move into Frozen and apply the slices.
//

use bevy::{app::App, ecs::schedule::SystemSet, prelude::*};
use tetris_rs::PausedState;

use crate::{
    game::{
        apply_slices, partitions, row_intersections, HitMap, Partitions,
        SliceRows,
    },
    image::{apply_slice_image, SliceImage},
    schedule::InGameSet,
};

/// Registers `row_intersections` and `partitions` in state
/// `PausedState::Playing` under system set `SliceSet`.
pub fn plugin(mut app: &mut App) {
    app.add_event::<SliceRows>()
        .add_event::<SliceImage>()
        .insert_resource(HitMap::default())
        .insert_resource(Partitions::default())
        .add_systems(
            Update,
            (
                //apply_slice_image,
                row_intersections,
                partitions,
            )
                .chain()
                .run_if(in_state(PausedState::Playing))
                .in_set(InGameSet::Slice),
        )
        .add_systems(
            PostUpdate,
            apply_slice_image.after(TransformSystem::TransformPropagate),
        )
        .observe(apply_slices);
}
