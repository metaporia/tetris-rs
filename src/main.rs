use bevy::prelude::*;
use bevy_prototype_lyon::prelude as bevy_lyon;
use bevy_prototype_lyon::prelude::*;
use bevy_rapier2d::prelude::*;

// width/height of single square
const BRICK_DIM: f32 = 30.0;
const PIXELS_PER_METER: f32 = 100.0;
const OUTLINE_THICKNESS: f32 = 3.0;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(
        WindowPlugin {
            primary_window: Some(Window {
                title: "Not Tetris".into(),
                name: Some("tetris-rs".into()),
                resolution: (BRICK_DIM * 12.0, BRICK_DIM * 19.0).into(),
                ..Default::default()
            }),
            ..Default::default()
        }))
        // Physics plugins
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(
            PIXELS_PER_METER,
        ))
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_plugins(ShapePlugin)
        // my plugins
        .add_systems(Startup, setup_graphics)
        .add_systems(Update, hello_world)
        //        .add_systems(Startup, setup_physics)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    // Add a camera so we can see the debug-render.
    commands.spawn(Camera2dBundle::default());
}

fn hello_world() {
    println!("hello_world");
}
