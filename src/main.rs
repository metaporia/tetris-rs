#![allow(unused, clippy::type_complexity, clippy::too_many_arguments)]
use std::{cmp::Ordering, collections::HashMap, f32::consts::PI};

use bevy::{prelude::*, window::WindowResolution};
use bevy_prototype_lyon::prelude::*;
use bevy_rapier2d::{geometry::RayIntersection, prelude::*};

use bevy_inspector_egui::quick::WorldInspectorPlugin;

mod arena;
pub mod debug;
pub mod freeze;
mod game;
pub mod image;
pub mod kbd;
pub mod row_density;
mod setup;
pub mod slice;
mod tetroid;

use arena::{spawn_arena, Ground};
use debug::DebugShape;
use game::{get_min_max, RowBounds, GROUND_Y, PIXELS_PER_METER};
use tetroid::components::*;
use tetroid::{spawn_tetromino, BRICK_DIM};

// width/height of single square
const IMPULSE_SCALAR: f32 = 20000.0;

fn main() {
    game::app();
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
    use bevy::{
        app::{App, Startup},
        ecs::system::ResMut,
    };
    use bevy_rapier2d::{
        plugin::{NoUserData, RapierContext, RapierPhysicsPlugin},
        rapier::dynamics::IntegrationParameters,
    };

    use crate::{tetroid::BRICK_DIM, PIXELS_PER_METER};

    pub(super) fn plugin(app: &mut App) {
        app.add_plugins(
            RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(
                PIXELS_PER_METER,
            ),
            //.with_length_unit(BRICK_DIM),
        )
        .add_systems(Startup, stiffen_contact_damping);
    }

    fn stiffen_contact_damping(mut rc: ResMut<RapierContext>) {
        // larger is softer: default = 5.0
        rc.integration_parameters.contact_damping_ratio = 0.1;
    }
}

mod graphics {
    //! Load TetrominoAssetPlugin, ShapePlugin
    use bevy::app::{App, Startup};
    use bevy_prototype_lyon::plugin::ShapePlugin;

    use crate::game::spawn_camera;
    use crate::image::TetrominoAssetPlugin;

    pub(super) fn plugin(app: &mut App) {
        app.add_plugins(TetrominoAssetPlugin)
            .add_plugins(ShapePlugin)
            .add_systems(Startup, spawn_camera);
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

pub mod tetromino {

    use bevy::{app::App, ecs::schedule::SystemSet, prelude::*};
    use tetris_rs::{AppState, PausedState};

    use crate::{
        game::{
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
