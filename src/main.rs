#![allow(unused, clippy::type_complexity, clippy::too_many_arguments)]
use std::{cmp::Ordering, collections::HashMap, f32::consts::PI};

use bevy::{prelude::*, window::WindowResolution};
use bevy_prototype_lyon::prelude::*;
use bevy_rapier2d::{geometry::RayIntersection, prelude::*};

use bevy_inspector_egui::quick::WorldInspectorPlugin;

mod arena;
mod event_demo;
pub mod image_demo;
mod setup;
mod tetroid;

use arena::{spawn_arena, Ground};
use debug_reset::DebugShape;
use event_demo::{get_min_max, RowBounds, GROUND_Y};
use tetroid::components::*;
use tetroid::{spawn_tetromino, BRICK_DIM};

// width/height of single square
const PIXELS_PER_METER: f32 = 10.0;

const IMPULSE_SCALAR: f32 = 20000.0;

fn main() {
    event_demo::app();
    //image_demo::fractal();
}

#[derive(Component)]
struct Row(u8);

#[derive(Event, Debug)]
pub struct Pause;

// sort by lowest y, if y's are equal lowest x
fn cmp_vec2_by_y(x: &Vec2, y: &Vec2) -> Ordering {
    if x.y < y.y {
        Ordering::Less
    } else if x.y > y.y {
        Ordering::Greater
    } else if x.x < y.x {
        Ordering::Less
    } else {
        Ordering::Equal
    }
}

fn v2_to_3(Vec2 { x, y }: &Vec2) -> Vec3 {
    Vec3::new(*x, *y, 0.0)
}

fn v3_to_2(Vec3 { x, y, .. }: &Vec3) -> Vec2 {
    Vec2::new(*x, *y)
}

/// Sort so that ordered traversal yields convex polygon.
///
/// See [stackoverflow post](https://stackoverflow.com/questions/73683410/sort-vertices-of-a-convex-polygon-in-clockwise-or-counter-clockwise) for algorithm
fn sort_convex_hull(points: &mut [Vec2]) {
    // get lowest, left most point
    if !points.is_empty() {
        points.sort_by(cmp_vec2_by_y);
        let lowest = points[0];
        let atan =
            |v: &Vec2| (v.y - lowest.y).atan2(v.x - lowest.x) + 2.0 * PI;
        points.sort_by(|a, b| atan(a).partial_cmp(&atan(b)).unwrap());
    }
}

fn draw_convex_hull(mut cmds: Commands, points: Vec<Vec2>, color: Color) {
    let convex_hull = shapes::Polygon {
        points,
        closed: true,
    };
    cmds.spawn((
        ShapeBundle {
            path: GeometryBuilder::build_as(&convex_hull),
            ..Default::default()
        },
        Stroke::new(color, 2.0),
        DebugShape,
    ));
}

/// Demo to test whether ray casts that hit at origin yield correct contact
/// point (they do if `solid==true`)
fn spawn_convex_hull(mut cmds: Commands, points: &[Vec2], color: Color) {
    // NOTE: `Collider::convex_hull` panics if `points < 2`
    if points.len() >= 2 {
        if let Some(convex_hull) = Collider::convex_hull(points) {
            info!("len = {}, points = {:?}", points.len(), points);
            cmds.spawn(RigidBody::Dynamic).insert(convex_hull);
        }
    }
}

// Circle contacts
// draw small circle at each intersection
fn draw_circle_contact(center: Vec2, mut cmds: Commands) {
    let contact_point = shapes::Circle {
        radius: 2.0,
        center,
    };
    cmds.spawn((
        ShapeBundle {
            path: GeometryBuilder::build_as(&contact_point),
            ..default()
        },
        Stroke::new(Color::Srgba(Srgba::GREEN), 1.0),
    ));
}

// Draw ray
fn draw_ray(mut cmds: Commands, origin: Vec2, contact_point: Vec2) {
    let ray = shapes::Line(origin, contact_point);
    cmds.spawn((
        ShapeBundle {
            path: GeometryBuilder::build_as(&ray),
            ..default()
        },
        Stroke::new(Color::Srgba(Srgba::WHITE), 1.0),
    ));
}

mod window {
    use bevy::{prelude::*, window::WindowResolution};

    use crate::tetroid::BRICK_DIM;
    pub(super) fn plugin(app: &mut App) {
        app.add_plugins(
            DefaultPlugins.build().set(WindowPlugin {
                primary_window: Some(Window {
                    title: "Not Tetris".into(),
                    name: Some("tetris-rs".into()),
                    resize_constraints: WindowResizeConstraints {
                        min_width: BRICK_DIM * 13.0,
                        min_height: BRICK_DIM * 19.0,
                        max_width: BRICK_DIM * 15.0,
                        max_height: BRICK_DIM * 26.0,
                    },
                    fit_canvas_to_parent: false,
                    // FIXME: fix ground alignment
                    resolution: WindowResolution::new(
                        BRICK_DIM * 15.0,
                        BRICK_DIM * 25.0,
                    )
                    .with_scale_factor_override(1.0),
                    ..Default::default()
                }),
                ..Default::default()
            }), //.disable::<LogPlugin>()
                //.set(LogPlugin {
                //filter: "tetris-rs=debug,bevy_ecs=debug".to_string(),
                //   ..Default::default()
                //}),
        );
    }
}

mod physics {
    //! Load rapier physics plugin
    use bevy::app::App;
    use bevy_rapier2d::plugin::{NoUserData, RapierPhysicsPlugin};

    use crate::PIXELS_PER_METER;

    pub(super) fn plugin(app: &mut App) {
        app.add_plugins(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(
            PIXELS_PER_METER,
        ));
    }
}

mod graphics {
    //! Load TetrominoAssetPlugin, ShapePlugin
    use bevy::app::{App, Startup};
    use bevy_prototype_lyon::plugin::ShapePlugin;

    use crate::event_demo::spawn_camera;
    use crate::image_demo::TetrominoAssetPlugin;

    pub(super) fn plugin(app: &mut App) {
        app.add_plugins(TetrominoAssetPlugin)
            .add_plugins(ShapePlugin)
            .add_systems(Startup, spawn_camera);
    }
}

pub mod debug_reset {
    use crate::event_demo::{FallSpeed, Freeze, SpawnNextTetromino, Tetroid};
    use bevy::{input::common_conditions::input_just_pressed, prelude::*};
    use bevy_prototype_lyon::entity::Path;
    use bevy_rapier2d::geometry::Collider;
    use tetris_rs::{AppState, PausedState};

    pub fn plugin(mut app: &mut App) {
        app.add_systems(
            Update,
            (
                reset_game.run_if(input_just_pressed(KeyCode::KeyR)),
                reset_tetroids.run_if(input_just_pressed(KeyCode::KeyT)),
                reset_debug_shapes.run_if(input_just_pressed(KeyCode::KeyC)),
            )
                .run_if(in_state(PausedState::Playing)),
        );
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

    // TODO: refactor spawning as event
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
}

pub mod menu {
    use bevy::{
        input::common_conditions::input_just_pressed, prelude::*,
        sprite::Anchor,
    };
    use tetris_rs::{AppState, GameState};

    use crate::util;

    // Exposes AppState::InitialGameSetup hook for one-time initialization,
    // e.g., spawning arena, the first tetroid, etc., before starting the game
    // loop.
    pub fn plugin(mut app: &mut App) {
        app.add_systems(OnEnter(AppState::MainMenu), greet)
            // enters AppState::InitialGameSetup when user presses space
            .add_systems(
                Update,
                initial_game_setup
                    .run_if(in_state(AppState::MainMenu))
                    .run_if(input_just_pressed(KeyCode::Space)),
            )
            .add_systems(
                OnExit(AppState::MainMenu),
                util::cleanup::<MenuCleanup>,
            )
            // Immediately leaves InitialGameSetup so that systems running in
            // this state run just once (I think).
            .add_systems(
                Update,
                finish_setup_and_start_game
                    .run_if(in_state(AppState::InitialGameSetup)),
            );
    }

    /// Move from MainMenu ->  InitialGameSetup
    fn initial_game_setup(
        mut app_state: ResMut<NextState<AppState>>,
        //mut game_state: ResMut<NextState<GameState>>,
    ) {
        info!("InitialGameSetup");
        app_state.set(AppState::InitialGameSetup);
    }

    fn finish_setup_and_start_game(
        mut app_state: ResMut<NextState<AppState>>,
    ) {
        app_state.set(AppState::InGame);
    }

    #[derive(Component)]
    struct MenuCleanup;

    fn greet(mut commands: Commands) {
        commands.spawn(MenuCleanup).insert(Text2dBundle {
            text: Text::from_section(
                "Press <space> to continue",
                TextStyle::default(),
            ),
            transform: Transform::from_xyz(0.0, 0.0, 0.0),
            text_anchor: Anchor::Center,
            ..default()
        });
    }
}

pub mod util {
    use bevy::prelude::*;

    pub fn cleanup<T: Component>(
        mut commands: Commands,
        entities: Query<Entity, With<T>>,
    ) {
        entities
            .iter()
            .for_each(|e| commands.entity(e).despawn_recursive())
    }
}

pub mod kbd_input {
    use crate::{ActiveTetromino, Pause, IMPULSE_SCALAR};
    use bevy::prelude::*;
    use bevy_rapier2d::dynamics::{
        Damping, ExternalForce, ExternalImpulse, Velocity,
    };
    use tetris_rs::{AppState, GameState, PausedState};

    pub fn plugin(mut app: &mut App) {
        app.add_event::<Pause>().add_systems(
            Update,
            (
                kbd_input.run_if(in_state(GameState::Playing)),
                (get_pause_input, toggle_pause)
                    .run_if(in_state(AppState::InGame)),
            ),
        );
    }

    /// Toggle the pouse state and virtual time.
    pub fn toggle_pause(
        mut time: ResMut<Time<Virtual>>,
        mut pause: EventReader<Pause>,
        mut next_state: ResMut<NextState<PausedState>>,
    ) {
        for p in pause.read() {
            if time.is_paused() {
                time.unpause();
                next_state.set(PausedState::Playing);
            } else {
                time.pause();
                next_state.set(PausedState::Paused);
            }
        }
    }

    /// Handle pause input in isolation in order to disable keyboard input while
    /// frozen.
    pub fn get_pause_input(
        kbd_input: Res<ButtonInput<KeyCode>>,
        mut pause: EventWriter<Pause>,
    ) {
        if kbd_input.just_pressed(KeyCode::Escape) {
            pause.send(Pause);
        }
    }

    /// Handle all keyboard input but Esc (which is handled by `get_pause_input`).
    pub fn kbd_input(
        kbd_input: Res<ButtonInput<KeyCode>>,
        mut ext_impulses: Query<
            (
                &mut ExternalImpulse,
                &mut ExternalForce,
                &mut Velocity,
                &mut Damping,
            ),
            With<ActiveTetromino>,
        >,
        //active_tetroid_query: Query<Entity, With<ActiveTetroid>>,
    ) {
        let Ok((mut ext_impulse, mut ext_force, mut velocity, mut damping)) =
            ext_impulses.get_single_mut()
        else {
            return;
        };

        let force = 2000000.0;
        let torque = force * 15.0;
        let max_vel = 230.0;
        let max_ang_vel = 2.7;

        let mut torque_impulse = 0.0;
        let torque_imp = 13.0 * IMPULSE_SCALAR;
        if kbd_input.pressed(KeyCode::KeyZ) && velocity.angvel < max_ang_vel {
            ext_force.torque = torque;
            //velocity.angvel = -max_ang_vel;
        }
        // FIXME: enable faster max angular velocity but with less momentum (reduce
        // collider mass).
        //
        //
        // - put `ColliderMassProperties` on rigibbody
        // - remove `ColliderMassProperties` from colliders
        // - ideally we'd like them all to rotate identicially, so identical
        //   angular inertia & center of mass. Hopefully a point mass will do this.

        // - NOTE: remove all forces on `DeactivateTetroid`
        else if kbd_input.pressed(KeyCode::KeyX)
            && velocity.angvel > -max_ang_vel
        {
            ext_force.torque = -torque;
            //velocity.angvel = max_ang_vel;
        } else {
            ext_force.torque = 0.0;
        }

        let mut impulse = Vec2::new(0.0, 0.0);
        let lateral_imp = 3.0 * IMPULSE_SCALAR;
        let vertical_imp = lateral_imp;

        // cap angular and linear velocities
        // otherwise apply impulses
        if kbd_input.pressed(KeyCode::ArrowLeft)
            && velocity.linvel.x > -max_vel
        {
            ext_force.force.x = -force;
        } else if kbd_input.pressed(KeyCode::ArrowRight)
            && velocity.linvel.x < max_vel
        {
            ext_force.force.x = force;
        } else {
            ext_force.force.x = 0.0;
        }

        let max_boost_vel = -300.0;
        if kbd_input.pressed(KeyCode::ArrowDown) {
            if velocity.linvel.y > max_boost_vel {
                ext_force.force.y = -force
            }
        } else {
            ext_force.force.y = 0.0;
            // TODO: apply damping if above speed limit.
            //if velocity.linvel.y >
            if velocity.linvel.y < max_boost_vel {
                damping.linear_damping = 5.0;
            } else {
                damping.linear_damping = 0.0;
            }
            // slow down block until it reaches 250.
        }
    }
}

pub mod slice {
    //! Handles ray casting (see `row_intersections`), partitioning (see
    //! `partitions`), and exposes a system set for the row density plugin to
    //! hook into.

    use bevy::{app::App, ecs::schedule::SystemSet, prelude::*};
    use tetris_rs::PausedState;

    use crate::{
        event_demo::{
            apply_slices, partitions, row_intersections, HitMap, Partitions,
            SliceRows,
        },
        image_demo::{apply_slice_image, SliceImage},
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
                (apply_slice_image, row_intersections, partitions)
                    .chain()
                    .run_if(in_state(PausedState::Playing))
                    .in_set(InGameSet::Slice),
            )
            .observe(apply_slices);
    }
}

pub mod tetromino {

    use bevy::{app::App, ecs::schedule::SystemSet, prelude::*};
    use tetris_rs::{AppState, PausedState};

    use crate::{
        event_demo::{
            active_tetromino_collisions, deactivate_tetromino,
            despawn_tetrominos, handle_active_tetromino_hit,
            spawn_next_tetromino, ActiveTetrominoHit, DeactivateTetromino,
            DespawnTetromino, SpawnNextTetromino,
        },
        schedule::InGameSet,
        tetroid::spawn_tetromino,
    };

    /// Registes tetromino spawn/despawn, deactivation, and collision detection
    /// systems.
    pub fn plugin(mut app: &mut App) {
        app.add_event::<ActiveTetrominoHit>()
            .add_event::<DeactivateTetromino>()
            .add_event::<SpawnNextTetromino>()
            .add_event::<DespawnTetromino>()
            .add_systems(OnEnter(AppState::InGame), spawn_tetromino)
            .add_systems(
                Update,
                (active_tetromino_collisions, handle_active_tetromino_hit)
                    .chain()
                    //.in_set(TetrominoCollisionSet)
                    .in_set(InGameSet::Collision)
                    .run_if(in_state(PausedState::Playing)), //.before(slice::SliceSet),
            )
            // TODO: Put despawn in PostUpdate, as it's an cleanup implementation detail
            .add_systems(
                Update,
                despawn_tetrominos
                    .run_if(in_state(PausedState::Playing))
                    .in_set(InGameSet::CleanUp),
                //.after(TetrominoCollisionSet),
            )
            .observe(spawn_next_tetromino)
            .observe(deactivate_tetromino);
    }
}

pub mod row_density {

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
}

pub mod freeze {
    use bevy::{
        app::{App, Plugin},
        prelude::*,
        time::{Timer, TimerMode},
    };
    use tetris_rs::GameState;

    use crate::event_demo::{freeze, unfreeze};

    /// Expects GameState to already have been initialized.
    pub fn plugin(app: &mut App) {
        // TODO: add `blink_row` that gets the current SliceRows
        // and blinks `em
        app.insert_resource(FreezeTimer::new())
            .add_systems(
                Update,
                FreezeTimer::tick.run_if(in_state(GameState::Frozen)),
            )
            .add_systems(
                OnEnter(GameState::Frozen),
                (FreezeTimer::init, freeze),
            )
            .add_systems(OnExit(GameState::Frozen), unfreeze);
    }

    // START Freeze logic
    #[derive(Resource)]
    struct FreezeTimer(Timer);

    impl FreezeTimer {
        pub fn new() -> Self {
            FreezeTimer(Timer::from_seconds(0.6, TimerMode::Repeating))
        }

        // frozen state time
        // - we could instead put the timer in the state enum so it only exists in the
        //   `Frozen` state
        fn init(mut timer: ResMut<FreezeTimer>) {
            // reset timer
            timer.0.reset();
            timer.0.unpause();
        }

        // tick timer until it finishes and then exit state
        fn tick(
            time: Res<Time>,
            mut timer: ResMut<FreezeTimer>,
            mut next_state: ResMut<NextState<GameState>>,
        ) {
            if timer.0.tick(time.delta()).just_finished() {
                dbg!(timer.0.remaining());
                next_state.set(GameState::Playing);
            }
        }
    }
}

pub mod schedule {
    use bevy::{app::App, prelude::*};

    #[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
    pub enum InGameSet {
        Collision,
        Slice,
        CleanUp,
    }

    pub fn plugin(app: &mut App) {
        app.configure_sets(
            Update,
            (InGameSet::Collision, InGameSet::Slice, InGameSet::CleanUp)
                .chain(),
        );
    }
}
