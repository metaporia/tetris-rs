use crate::{
    game::{FallSpeed, Freeze, SpawnNextTetromino, Tetroid},
    tetroid::TetrominoCollider,
    ActiveTetromino,
};
use bevy::{input::common_conditions::input_just_pressed, prelude::*};
use bevy_prototype_lyon::entity::Path;
use bevy_rapier2d::geometry::{Collider, ColliderMassProperties};
use tetris_rs::{AppState, PausedState};

pub fn plugin(mut app: &mut App) {
    app.add_systems(
        Update,
        (
            reset_game.run_if(input_just_pressed(KeyCode::KeyR)),
            reset_tetroids.run_if(input_just_pressed(KeyCode::KeyT)),
            reset_debug_shapes.run_if(input_just_pressed(KeyCode::KeyC)),
            dbg_mass_props.run_if(input_just_pressed(KeyCode::Space)),
        )
            .run_if(in_state(PausedState::Playing)),
    );
}

/// Should fire on space key
fn dbg_mass_props(
    at: Query<(Entity, &Children), With<ActiveTetromino>>,
    mass_props: Query<
        (Entity, &ColliderMassProperties),
        With<TetrominoCollider>,
    >,
) {
    let Ok((at, children)) = at.get_single() else {
        return;
    };

    children.iter().for_each(|c| {
        let Ok((_, mass_props)) = mass_props.get(*c) else {
            return;
        };

        dbg!(at, mass_props);
    });
}

fn reset_game(
    mut cmds: Commands,
    tetroids: Query<Entity, With<Tetroid>>,
    shapes: Query<
        Entity,
        (With<Path>, With<Handle<ColorMaterial>>, Without<Collider>),
    >,
    mut freeze: EventReader<Freeze>,
    mut images: ResMut<Assets<Image>>,
    fallspeed: Res<FallSpeed>,
    mut next_tetroid: EventWriter<SpawnNextTetromino>,
) {
    freeze.clear();
    // despawn
    tetroids
        .iter()
        .for_each(|t| cmds.entity(t).despawn_recursive());
    shapes
        .iter()
        .for_each(|s| cmds.entity(s).despawn_recursive());

    //spawn_tetromino(cmds, images, fallspeed);
    cmds.trigger(SpawnNextTetromino);
    //next_tetroid.send(SpawnNextTetromino);
}

fn reset_tetroids(
    mut cmds: Commands,
    tetroids: Query<Entity, With<Tetroid>>,
    mut freeze: EventReader<Freeze>,
    mut images: ResMut<Assets<Image>>,
    fallspeed: Res<FallSpeed>,
    mut next_tetroid: EventWriter<SpawnNextTetromino>,
) {
    freeze.clear();
    // despawn
    tetroids
        .iter()
        .for_each(|t| cmds.entity(t).despawn_recursive());
    //next_tetroid.send(SpawnNextTetromino);
    cmds.trigger(SpawnNextTetromino);
    //spawn_tetromino(cmds, images, fallspeed);
}

#[derive(Component)]
pub(crate) struct DebugShape;

fn reset_debug_shapes(
    mut cmds: Commands,
    debug_shapes: Query<Entity, With<DebugShape>>,
    mut freeze: EventReader<Freeze>,
    mut images: ResMut<Assets<Image>>,
    fallspeed: Res<FallSpeed>,
    mut next_tetroid: EventWriter<SpawnNextTetromino>,
) {
    freeze.clear();
    // despawn
    debug_shapes
        .iter()
        .for_each(|t| cmds.entity(t).despawn_recursive());
    //spawn_tetromino(cmds, images, fallspeed);
    cmds.trigger(SpawnNextTetromino);
    //next_tetroid.send(SpawnNextTetromino);
}
