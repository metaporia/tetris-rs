#![allow(unused, clippy::type_complexity)]
use bevy::{prelude::*, transform, window::WindowResolution};
use bevy_prototype_lyon::prelude::*;
use bevy_rapier2d::{
    parry::query::details::contact_manifolds_trimesh_shape_shapes, prelude::*,
    rapier::geometry::ActiveCollisionTypes,
};

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
                    resolution: WindowResolution::new(
                        BRICK_DIM * 9.0,
                        BRICK_DIM * 13.0,
                    )
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
        .add_systems(Update, kbd_input)
        // test collider
        .add_systems(Update, freeze_on_ground_contact)
        //.add_systems(Startup, test_collider_setup_sensor)
        //.add_systems(Update, test_collider_ray_cast)
        .add_systems(Startup, row_density_solver_setup)
        .add_systems(Update, row_collisions)
        //.add_systems(Update, test_collider_ray_cast)
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

fn _move_block(
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

fn _rotate_block2(
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

fn kbd_input(
    kbd_input: Res<ButtonInput<KeyCode>>,
    mut ext_impulses: Query<&mut ExternalImpulse, With<ActiveTetroid>>,
    mut hit_ground_events: EventReader<HitGround>,
    active_tetroid_query: Query<Entity, With<ActiveTetroid>>,
) {
    let active_tetroid = active_tetroid_query.get_single();
    match active_tetroid {
        Err(_) => {}
        Ok(at) => match hit_ground_events.read().find(|hg| hg.0 == at) {
            Some(_) => {}
            _ => {
                let mut impulse = 0.0;
                let imp = 5.0 * IMPULSE_SCALAR;
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
        },
    }
}

/// On ground contact remove all velocity & gravity for ActiveTetroid, remove ActiveTetroid from entity
///
/// Event order is:
/// HitGround -> UpdateDensity -> Slice -> UnfreezeDebris
fn freeze_on_ground_contact(
    mut commands: Commands,
    mut event_writer: EventWriter<HitGround>,
    mut collision_events: EventReader<CollisionEvent>,
    mut ground_query: Query<Entity, (With<Collider>, With<Ground>)>,
    mut active_tetroid_query: Query<
        (Entity, &mut Velocity, &mut GravityScale),
        (With<Collider>, With<ActiveTetroid>),
    >,
) {
    match active_tetroid_query.get_single_mut() {
        Err(_) => {}
        Ok((at, mut vel, mut gs)) => {
            let g = ground_query.single_mut();

            for ce in collision_events.read() {
                if let CollisionEvent::Started(e1, e2, _) = ce {
                    // active tetroid has hit ground
                    if (*e1 == at && *e2 == g) || (*e2 == at && *e1 == g) {
                        // 1. freeze block on collision
                        vel.linvel = Vect::ZERO;
                        vel.angvel = 0.0;
                        gs.0 = 0.0;

                        event_writer.send(HitGround(at));
                        // remove ActiveTetroid component from just-collided entity
                        //commands.entity(at).remove::<ActiveTetroid>();
                    }
                }
            }
        }
    }
}

#[derive(Component)]
struct Row;

// Triggered by HitGround in order to visually confirm collision results
// need to add solver group that ensures RowSensors cannot collide with any tetroids
fn row_density_solver_setup(mut cmds: Commands) {
    // add collider
    cmds.spawn(Collider::cuboid(BRICK_DIM * 5.0, BRICK_DIM / 2.0))
        .insert(ActiveEvents::COLLISION_EVENTS)
        .insert(SolverGroups::new(Group::GROUP_1, Group::NONE))
        .insert(RigidBody::Fixed)
        .insert(Row)
        .insert(TransformBundle::from(Transform::from_xyz(
            0.0,
            -BRICK_DIM * (18.0 / 2.0) + BRICK_DIM,
            0.0,
        )));
}

fn row_collisions(
    mut cmds: Commands,
    transform_query: Query<&GlobalTransform>,
    rapier_context: Res<RapierContext>,
    mut collisions: EventReader<CollisionEvent>,
    row_query: Query<Entity, With<Row>>,
    active_tetroid_query: Query<Entity, With<ActiveTetroid>>,
) {
    // for c in collisions.read() {
    //     if let CollisionEvent::Started(e1, e2, _) = c {
    //         println!("collision event: {:?}", c);
    //     }
    // }

    if let Ok(at) = active_tetroid_query.get_single() {
        if let Ok(row) = row_query.get_single() {
            for c in collisions.read() {
                // FIXME: i think we can just query the collision with contact pair, no?
                if let CollisionEvent::Stopped(e1, e2, _) = c {
                    if (*e1 == row || *e2 == row) {
                        if let Some(contact_pair) =
                          // can use ConttackPa
                            rapier_context.contact_pair(*e1, *e2)
                        {
                            // Make sure that the collider's transform is used
                            // to transform the contact point from local space.
                            // TODO may not be necessary
                            let row_collider =
                                if contact_pair.collider1() == row {
                                    contact_pair.collider1()
                                } else {
                                    contact_pair.collider2()
                                };
                            for manifold in contact_pair.manifolds() {
                                //if let Ok(transform) =
                                //    transform_query.get(row_collider)
                                //{
                                //    if (*e2 == row) { println!("e2 is row");}
                                //    let row_local_n =
                                //        if contact_pair.collider1() == row {
                                //            manifold.local_n1()
                                //        } else {
                                //            manifold.local_n2()
                                //        };
                                //    // convert row_local_n to global space
                                //    println!("contact normal 2: {:?}", manifold.local_n2());
                                //    let gcp = transform
                                //        .transform_point(Vec3::new(
                                //            manifold.local_n2().x,
                                //            manifold.local_n2().y,
                                //            0.0,
                                //        ));

                                //    // collider1 is row
                                //    circle_contact(Vec2::new(gcp.x, gcp.y), &mut cmds)
                                //}
                                // TODO use manifold.find_deepest_contact
                                // contact graph
                                for p in manifold.points() {
                                    // not all manifold points are actually in
                                    // contact. If point is greater than 4
                                    // units away, don't render it.
                                   if (p.dist().abs() > 5.0 ) { continue; };
                                    // FIXME we need the transform of the local space to
                                    // draw the circles
                                    if let Ok(transform) =
                                        transform_query.get(*e1)
                                    {
                                        // draw centerpoints of colliders
                                        //let pt = Vec2::new(transform.translation.x, transform.translation.y);
                                        //circle_contact(pt+p.local_p1(), &mut cmds);
                                    }
                                    if let Ok(transform) =
                                        transform_query.get(*e2)
                                    {
                                        if (*e2 == row) { println!("e2 is row");}
                                        // draw centerpoints of colliders
                                        // TODO apply rotation and scale
                                        let Vec2 { x, y } = p.local_p2();
                                        // apply transform to local contact point to get it in
                                        // global space
                                        let Vec3 { x, y, .. } = transform
                                            .transform_point(Vec3::new(
                                                x, y, 0.0,
                                            ));
                                        let global_contact_point =
                                            Vec2::new(x, y);
                                        circle_contact(
                                            global_contact_point,
                                            &mut cmds,
                                        );
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

fn test_collider_setup_sensor(mut commands: Commands) {
    // create collider sensor on first row to slice brick once it hits the
    // bottom
    commands
        .spawn(Collider::cuboid(BRICK_DIM * 5.0, BRICK_DIM / 2.0))
        .insert(Sensor)
        .insert(Row)
        .insert(TransformBundle::from(Transform::from_xyz(
            0.0,
            -BRICK_DIM * (18.0 / 2.0) + BRICK_DIM,
            0.0,
        )));
}

// TODO circle all concat points between ACtiveTetroid and RowSensor
// FIXME: test_collider rewrite: demo for row density calculations
// - each row is a `RigidBody::Fixed` and we add it to a solver_group, say,
// `Row`, that will not have contact forces generated for it when in contact
// with any Tetroid (active or otherwise).
// -
//
//
fn test_collider_ray_cast(
    mut commands: Commands,
    mut collision_events: EventReader<CollisionEvent>,
    mut row_sensor_query: Query<Entity, With<Row>>,
    mut tetroid_query: Query<
        (
            Entity,
            &mut Transform,
            &mut Collider,
            &mut Velocity,
            &mut GravityScale,
        ),
        With<ActiveTetroid>,
    >,
    rapier_context: Res<RapierContext>,
) {
    // for this demo, the block freezes when it hits the lowest row
    // then the ray is sent from the top of the second lowest to find the
    // points of intersection
    //
    // cast ray to calculate intersection points
    let ray_origin = Vec2::new(BRICK_DIM * 5.0, BRICK_DIM * -6.5);
    let ray_dir = Vec2::new(-1.0, 0.0);
    let max_toi = 200.0;

    let mut ray_destination = ray_origin;
    ray_destination.x *= -1.0;

    // let's try shape intersection

    for ce in collision_events.read() {
        if let CollisionEvent::Started(e0, e1, flags) = ce {
            // determine which is activeblock
            let (entity, transform, collider, mut velocity, mut gravity_scale) =
                tetroid_query.single_mut();
            println!(
                "id: {}, e0.translation: {}, e0.rotation: {}",
                entity.index(),
                transform.translation,
                transform.rotation
            );

            for pair_view in rapier_context.contact_pairs_with(entity) {
                println!(
                    "{:?} hits {:?}",
                    pair_view.collider1(),
                    pair_view.collider2()
                );
            }

            let callback = |hit_entity, ray_intersection: RayIntersection| {
                //println!(
                //    "hit ent {:?} at point {:?}",
                //    hit_entity, ray_intersection.point
                //);
                // TODO spawn line/rectangle

                // draw small circle at each intersection
                //commands.spawn(circle_contact(ray_intersection.point));
                circle_contact(ray_intersection.point, &mut commands);
                true
            };

            rapier_context.intersections_with_ray(
                ray_origin,
                ray_dir,
                max_toi,
                false, // solid?
                QueryFilter::only_dynamic(),
                callback,
            );

            //if let Some(pair_view) = rapier_context.contact_pair(*e0, *e1) {
            //    pair_view
            //        .manifolds()
            //        .for_each(|m| {
            //            m.rigid_body1().map(|e| println!("local_p1 ent id: {:?}", e));
            //            m.rigid_body2().map(|e| println!("local_p2 ent id: {:?}", e));
            //            m.points().for_each(|p| println!("point: {}", p.local_p1()))
            //        });
            //}

            // 1. freeze block on collision
            //velocity.linvel = Vect::ZERO;
            //velocity.angvel = 0.0;
            //gravity_scale.0 = 0.0;
            // 2. slice and dice
            // 3. convert from ActiveTetroid to TetroidDebris (which should
            //    be a side effect of despawning the entity and spawning
            //    the calculated fragments
        }
    }
}

// Circle contacts
// draw small circle at each intersection
fn circle_contact(center: Vec2, cmds: &mut Commands) {
    let contact_point = shapes::Circle {
        radius: 2.0,
        center,
    };
    cmds.spawn((
        ShapeBundle {
            path: GeometryBuilder::build_as(&contact_point),
            ..default()
        },
        Stroke::new(Color::RED, 1.0),
    ));
}
