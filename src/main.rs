use bevy::{prelude::*, window::WindowResolution};
use bevy_prototype_lyon::prelude::*;
use bevy_rapier2d::prelude::*;

// width/height of single square
const BRICK_DIM: f32 = 30.0;
const PIXELS_PER_METER: f32 = 100.0;
const OUTLINE_THICKNESS: f32 = 3.0;

const IMPULSE_SCALAR: f32 = 10000.0;

fn main() {
    App::new()
        .add_event::<HitGround>()
        .add_plugins(
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    title: "Not Tetris".into(),
                    name: Some("tetris-rs".into()),
                    // FIXME: fix ground alignment
                    resolution: WindowResolution::new(BRICK_DIM * 9.0, BRICK_DIM * 13.0)
                        .with_scale_factor_override(1.0),
                    ..Default::default()
                }),
                ..Default::default()
            }),
        )
        // Physics plugins
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(
            PIXELS_PER_METER,
        ))
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_plugins(ShapePlugin)
        // my plugins
        .add_systems(Startup, setup_graphics)
        //        .add_systems(Startup, setup_physics)
        .add_systems(Startup, spawn_arena)
        .add_systems(Startup, spawn_lblock)
        // TODO Update or FixedUpdate?
        .add_systems(Update, rotate_block)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    // Add a camera so we can see the debug-render.
    commands.spawn(Camera2dBundle::default());
}

#[derive(Event)]
struct HitGround(Entity);

/* Single Block plugin */
/// Nominal component used to label actively falling tetroid.
#[derive(Component)]
struct ActiveTetroid;

#[derive(Component)]
struct LBlock;

#[derive(Component)]
struct Ground;

fn spawn_lblock(mut commands: Commands) {
    let lblock_pts = vec![
        // drawns "L" block
        Vect { x: 0.0, y: 0.0 },
        Vect {
            x: 0.0,
            y: BRICK_DIM * 3.0,
        },
        Vect {
            x: BRICK_DIM,
            y: BRICK_DIM * 3.0,
        },
        Vect {
            x: BRICK_DIM,
            y: BRICK_DIM,
        },
        Vect {
            x: BRICK_DIM * 2.0,
            y: BRICK_DIM,
        },
        Vect {
            x: BRICK_DIM * 2.0,
            y: 0.0,
        },
    ];
    let lblock = shapes::Polygon {
        points: lblock_pts.clone(),
        closed: true,
    };

    commands
        .spawn((
            ShapeBundle {
                path: GeometryBuilder::build_as(&lblock),
                ..default()
            },
            Fill::color(Color::CYAN),
            Stroke::new(Color::BLACK, 2.0),
        ))
        .insert(RigidBody::Dynamic)
        .insert(Collider::convex_decomposition(
            &lblock_pts,
            // list of segments as vertex pairs
            &[[0, 1], [1, 2], [2, 3], [3, 4], [4, 5], [5, 0]],
        ))
        .insert(Restitution::coefficient(0.2))
        //.insert(TransformBundle::from(Transform::from_xyz(0.0, 320.0, 0.0)))
        .insert(TransformBundle::from(Transform::from_xyz(
            -BRICK_DIM, 320.0, 0.0,
        )))
        .insert(ExternalImpulse {
            impulse: Vec2::new(0.0, 0.0),
            torque_impulse: 0.0,
        })
        .insert(ActiveEvents::COLLISION_EVENTS)
        .insert(LBlock)
        .insert(ActiveTetroid)
        .insert(Velocity {
            linvel: Vec2::new(0.0, -120.0),
            angvel: 0.0,
        })
        .insert(GravityScale(0.02))
        .log_components();
}

fn spawn_arena(mut commands: Commands) {
    let brick_half: f32 = BRICK_DIM / 2.0;

    let ground_extents = Vect {
        x: BRICK_DIM * 5.0,
        y: BRICK_DIM / 2.0,
    };
    let ground_origin_y = -BRICK_DIM * (18.0 / 2.0);
    let ground_shape = shapes::Rectangle {
        extents: Vect {
            x: ground_extents.x * 2.0 - OUTLINE_THICKNESS,
            y: ground_extents.y * 2.0 - OUTLINE_THICKNESS,
        },
        ..Default::default()
    };
    /* Create the ground. */
    commands
        .spawn((
            ShapeBundle {
                path: GeometryBuilder::build_as(&ground_shape),
                ..default()
            },
            Fill::color(Color::BLACK),
            Stroke::new(Color::BLACK, OUTLINE_THICKNESS),
        ))
        .insert(RigidBody::Fixed)
        .insert(Collider::cuboid(ground_extents.x, ground_extents.y))
        .insert(TransformBundle::from(Transform::from_xyz(
            0.0,
            ground_origin_y,
            0.0,
        )))
        .insert(Ground);

    /* Create left wall */
    let wall_shape = shapes::Rectangle {
        extents: Vect {
            x: BRICK_DIM - OUTLINE_THICKNESS,
            y: BRICK_DIM * 19.0 - OUTLINE_THICKNESS,
        },
        ..Default::default()
    };
    commands
        .spawn((
            ShapeBundle {
                path: GeometryBuilder::build_as(&wall_shape),
                ..default()
            },
            Fill::color(Color::BLACK),
            Stroke::new(Color::BLACK, OUTLINE_THICKNESS),
        ))
        .insert(Collider::cuboid(brick_half, BRICK_DIM * 9.5))
        .insert(TransformBundle::from(Transform::from_xyz(
            -BRICK_DIM * 5.0 - brick_half,
            0.0,
            0.0,
        )));

    // right wall
    commands
        .spawn((
            ShapeBundle {
                path: GeometryBuilder::build_as(&wall_shape),
                ..default()
            },
            Fill::color(Color::BLACK),
            Stroke::new(Color::BLACK, OUTLINE_THICKNESS),
        ))
        .insert(Collider::cuboid(brick_half, BRICK_DIM * 9.5))
        .insert(TransformBundle::from(Transform::from_xyz(
            BRICK_DIM * 5.0 + brick_half,
            0.0,
            0.0,
        )));
}

fn move_block(
    kbd_input: Res<ButtonInput<KeyCode>>,
    mut impulses: Query<&mut ExternalImpulse, With<LBlock>>,
) {
    for mut imp in impulses.iter_mut() {
        // NOTE: impulse numbers need to be high due to reduced gravity I guess
        if kbd_input.pressed(KeyCode::ArrowRight) {
            println!("right");
            imp.impulse = Vec2::new(IMPULSE_SCALAR, 0.0)
        }

        if kbd_input.pressed(KeyCode::ArrowLeft) {
            imp.impulse = Vec2::new(-IMPULSE_SCALAR, 0.0)
        }
    }
}

fn rotate_block2(
    kbd_input: Res<ButtonInput<KeyCode>>,
    mut ext_impulses: Query<&mut ExternalImpulse, With<LBlock>>,
) {
    let mut impulse = 0.0;
    let imp = 0.5 * IMPULSE_SCALAR;
    if kbd_input.pressed(KeyCode::KeyZ) {
        impulse = imp;
    } else if kbd_input.pressed(KeyCode::KeyX) {
        impulse = -imp;
    }

    let mut lateral_impulse = 0.00;
    let lateral_imp = 0.05 * IMPULSE_SCALAR;
    if kbd_input.pressed(KeyCode::ArrowLeft) {
        lateral_impulse = -lateral_imp;
    }
    if kbd_input.pressed(KeyCode::ArrowRight) {
        lateral_impulse = lateral_imp;
    }
    for mut ext_impulse in ext_impulses.iter_mut() {
        //println!(
        //    "impulse: {}, lateral_impulse: {}",
        //    &impulse, &lateral_impulse
        //);
        //println!(
        //    "torque: {}, lateral: {}",
        //    &ext_impulse.torque_impulse, &ext_impulse.impulse
        //);
        ext_impulse.torque_impulse = impulse;
        ext_impulse.impulse = Vec2::new(lateral_impulse, 0.0);
    }
}

fn rotate_block(
    kbd_input: Res<ButtonInput<KeyCode>>,
    mut ext_impulses: Query<&mut ExternalImpulse, With<ActiveTetroid>>,
    mut hit_ground_events: EventReader<HitGround>,
    active_tetroid_query: Query<Entity, With<ActiveTetroid>>,
) {
    let active_tetroid = active_tetroid_query.single();
    match hit_ground_events.read().find(|hg| hg.0 == active_tetroid) {
        Some(_) => {}
        _ => {
            let mut impulse = 0.0;
            let imp = 0.05 * IMPULSE_SCALAR;
            if kbd_input.pressed(KeyCode::KeyZ) {
                impulse = imp;
            } else if kbd_input.pressed(KeyCode::KeyX) {
                impulse = -imp;
            }

            let mut lateral_impulse = 0.00;
            let lateral_imp = 5.0 * IMPULSE_SCALAR;
            if kbd_input.pressed(KeyCode::ArrowLeft) {
                lateral_impulse = -lateral_imp;
            }
            if kbd_input.pressed(KeyCode::ArrowRight) {
                lateral_impulse = lateral_imp;
            }
            for mut ext_impulse in ext_impulses.iter_mut() {
                println!(
                    "impulse: {}, lateral_impulse: {}",
                    &impulse, &lateral_impulse
                );
                println!(
                    "torque: {}, lateral: {}",
                    &ext_impulse.torque_impulse, &ext_impulse.impulse
                );
                ext_impulse.torque_impulse = impulse;
                ext_impulse.impulse = Vec2::new(lateral_impulse, 0.0);
            }
        }
    }
}
