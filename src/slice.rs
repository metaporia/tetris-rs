//! Handles ray casting (see `row_intersections`), partitioning (see
//! `partitions`), and exposes a system set for the row density plugin to
//! hook into.

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

// FIXME: Not sure if it was just when manually slicing with `send_slice` but
// there are cases where group_hulls is failing.
// - To test we need to isolate a failing collider, and dump the collider
//   points to see how `any_shared_points` is fucking up.

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
