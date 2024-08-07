use bevy::{app::App, ecs::schedule::SystemSet, prelude::*};
use tetris_rs::{AppState, PausedState};

use crate::{
    arena::{
        check_row_densities, clear_row_densities, render_row_density,
        ClearRowDensities, RowDensity, RowDensityIndicatorMap,
    },
    schedule::InGameSet,
};

pub fn plugin(mut app: &mut App) {
    app.add_event::<RowDensity>()
        .add_event::<ClearRowDensities>()
        .insert_resource(RowDensityIndicatorMap::default())
        .add_systems(
            Update,
            (check_row_densities, render_row_density)
                //.in_set(RowDensityCheckSet)
                .in_set(InGameSet::CleanUp)
                //.after(slice::SliceSet))
                .run_if(in_state(PausedState::Playing)),
        )
        .observe(clear_row_densities);
}
