#![allow(unused, clippy::type_complexity)]
use bevy::{prelude::*, transform, window::WindowResolution};
use bevy_prototype_lyon::prelude::*;
use bevy_rapier2d::{
    parry::query::details::contact_manifolds_trimesh_shape_shapes,
    prelude::*,
    rapier::geometry::{ActiveCollisionTypes, RayIntersection},
};

use bevy::gizmos::prelude::*;

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
        .add_systems(Update, test_collider_ray_cast)
        //.add_systems(Startup, row_density_solver_setup)
        //.add_systems(Update, row_collisions)
        .add_systems(Update, cast_rays)
        // Debug ephemeral rendering
        .add_systems(Update, gizmo_test)
        .add_systems(Update, draw_vertices)
        .run();
}

fn gizmo_test(mut gizmos: Gizmos) {
    gizmos.line(Vec3::ZERO, Vec3::new(30.0, 30.0, 10.0), Color::GREEN);
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
struct Row;

#[derive(Component)]
struct LBlock;

#[derive(Component)]
struct Ground;

#[derive(Component, Debug)]
struct Vertices {
    vs: Vec<Vec2>,
}

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
        // try with Polyline instead of convex convex decomposition
        .insert(Collider::convex_decomposition(
            &lblock_pts,
            // list of segments as vertex pairs
            &[[0, 1], [1, 2], [2, 3], [3, 4], [4, 5], [5, 0]],
        ))
        .insert(Vertices {
            vs: lblock_pts.clone(),
        })
        //.insert(Collider::polyline(
        //    lblock_pts.clone(),
        //    // list of segments as vertex pairs
        //    Some(vec![[0, 1], [1, 2], [2, 3], [3, 4], [4, 5], [5, 0]]),
        //))
        .insert(Restitution::coefficient(0.2))
        .insert(Ccd::enabled()) // enable continous collision detection
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

// FIXME: test_collider rewrite: demo for row density calculations
// TODO row density for arbitrary tetroid/debris
// ..
// NB. This is the general algorithm for any tetroid not just the ActiveTetroid
// (though the initial demo will use the ActiveTetroid).
// ..
// 1. freeze all tetroid/debris entities on collision of active tetroid with any
//    debris
// 2. for a given row, r, set up four ray casts:
// ..
//            | r0-->| |<----r3|
//            |      | |__     |
//            | r1-->|___|<--r2|
// FIXME:
// 3. find contacts for each and classify intersection of tetroid row
//    The tetroid/debris's intersection with the row can be classfied in one of six
//    ways:
//    1. Above (no contact/nominal point contact without overlap);
//    2. Below (no/nominal contact);
//    3. Inside, that is, no contact with rays but debris.y is between r0.y and r1.y
//    4. Double-intersection, contacts both upper and lower rays, will be cut into
//       three pieces.
//    5. Upper single-intersection, contacts only upper ray, yields only two parts.
//    6. Lower single-intersection, contacts only lower ray, yields only two parts.

// FIXME: classification procedure
// 1. Cast rays
// 2. Check contacts:
//    match # of collisions (filter out nominal collisions)
//      4 -> Double-intersection (4)
//      2 -> Single-intersection (5 or 6):
//             check variant:
//               if upper ray -> upper single-intersection (5)
//               if lower ray -> lower single-intersection (6)
//      1,3 -> error, this should not happen
//      0 -> Check if:
//           Inside (r1.y < debris.y < r0.y)
//           For Below & Above: loop through vertices of debris, find lowest &
//           highest y-coords.
// NOTE: Demo of Double-intersection case
fn cast_rays(
    mut cmds: Commands,
    rc: Res<RapierContext>,
    mut ev_hit_ground: EventReader<HitGround>,
) {
    // exit if no hit ground events
    if ev_hit_ground.is_empty() {
        return;
    }

    println!("cast_rays: received HitGround event");
    // board is 10 x 18

    let lu_origin = Vec2::new(BRICK_DIM * -5.0, BRICK_DIM * -6.5);
    let lb_origin = Vec2::new(BRICK_DIM * -5.0, BRICK_DIM * -7.5);
    let ru_origin = Vec2::new(BRICK_DIM * 5.0, BRICK_DIM * -6.5);
    let rb_origin = Vec2::new(BRICK_DIM * 5.0, BRICK_DIM * -7.5);
    let l_dir = Vec2::new(1.0, 0.0);
    let r_dir = Vec2::new(-1.0, 0.0);
    // From example, unclear what appropriate value for max_toi is for this
    // use-case
    // see https://rapier.rs/docs/user_guides/bevy_plugin/scene_queries/#ray-casting
    let max_toi = 300.0;
    let filter = QueryFilter::only_dynamic();

    // Cast rays -> classify (though this will *almost* always be a
    // Double-intersection).
    // left-upper

    let bundle_hit = move |origin: Vec2, dir: Vec2| {
        move |(entity, toi): (Entity, f32)| (entity, origin + dir * toi)
    };

    let bundle_hit_ray_intersection =
        move |origin: Vec2, dir: Vec2| move |(ent, ri)| (ent, ri);

    let mlu_hit = rc
        .cast_ray_and_get_normal(lu_origin, l_dir, max_toi, false, filter)
        .map(bundle_hit_ray_intersection(lu_origin, l_dir));
    let mlb_hit = rc
        .cast_ray(lb_origin, l_dir, max_toi, true, filter)
        .map(bundle_hit(lb_origin, l_dir));

    let mru_hit = rc
        .cast_ray(ru_origin, r_dir, max_toi, true, filter)
        .map(bundle_hit(ru_origin, r_dir));
    let mrb_hit = rc
        .cast_ray(rb_origin, r_dir, max_toi, true, filter)
        .map(bundle_hit(rb_origin, r_dir));

    match (mru_hit, mrb_hit, mlu_hit, mlb_hit) {
        (
            Some((ru_ent, ru_hit)),
            Some((rb_ent, rb_hit)),
            Some((lu_ent, lu_ri)),
            Some((lb_ent, lb_hit)),
        ) => {
            println!("cast_rays: Double-intersection");
            draw_circle_contact(ru_hit, cmds.reborrow());
            draw_ray(cmds.reborrow(), ru_origin, ru_hit);

            draw_circle_contact(rb_hit, cmds.reborrow());
            draw_ray(cmds.reborrow(), rb_origin, rb_hit);

            draw_circle_contact(lu_ri.point, cmds.reborrow());
            draw_ray(cmds.reborrow(), lu_origin, lu_ri.point);
            println!("cast_rays: FeatureId: {:?}", lu_ri.feature);
            match lu_ri.feature {
                bevy_rapier2d::parry::shape::FeatureId::Vertex(_) => {
                    println!("vertex")
                }
                bevy_rapier2d::parry::shape::FeatureId::Face(_) => {
                    println!("vertex")
                }
                bevy_rapier2d::parry::shape::FeatureId::Unknown => {
                    println!("vertex")
                }
            }

            draw_circle_contact(lb_hit, cmds.reborrow());
            draw_ray(cmds.reborrow(), lb_origin, lb_hit);
        }
        _ => {
            let x = 1;
        } // Skipping classification for now as it shouldn't be too difficult.
          // TODO: Identity which edge the contact point/hit lies on.
          // - See `geommetry::RayIntersection: FeatureId`
    }
}

// Let's see what kind of path/feature/segment data we can actually get from a
// collider to inform the density check.
// NOTE: just query the component `Vertices` to get the current vertices of the tetroid in local
// space.
fn draw_vertices(
    mut gizmos: Gizmos,
    mut cmds: Commands,
    mut q_tetroid: Query<
        (&mut Vertices, &GlobalTransform),
        With<ActiveTetroid>,
    >,
) {
    if let Ok((vs, transform)) = q_tetroid.get_single_mut() {
        let vs_global: Vec<Vec3> = vs
            .vs
            .iter()
            .map(|&Vec2 { x, y }| {
                transform.transform_point(Vec3::new(x, y, 0.0))
            })
            .collect();
    gizmos.linestrip(vs_global, Color::GREEN);  
    }
}

// FIXME: Find area: slice debris into comonent shapes (could cache them as
// components for when the actual slicing happens as the row density check will
// happen continuously? ehh)
// ..
// 1. Get list of vertices of debris (since it's modelled as a compound shape, we
//    either have to reassemble from `Iter<Vect, Rot, CompoundView>` (more hassle
//    but better parity with rapier state) OR use the debris' GlobalTransform and
//    initial vector (could attach this to the entity as component or get it from
//    the lyon `Shape`--let's try this to start)
//    So query ShapeBundle, get its lyon_path::Path, apply Transform of debris to
//    get the endpoints (vertices) in global space.
//    FIXME:
//    Ideally, we can start at an intersection point and loop through the vertices
//    to assemble each new shape.
//    - The ShapeBundle's Path provides an ordered list of endpoints/line
//    segments, so we can traverse them with an accumulator and whenever a
//    contact point is between between two adjacent vertices we have a new
//    vertex and collect vertices, e.g, for an upper slice, until the next
//    contact point.
//
// FIXME:
// Consider the following, which has a concavity (thus complicating things):
// We traverse endpoints, v[0..5] (which may start with an less-than-optimal
// vertex), with an accumulator, which has a triple of vectors to collect
// endpoints, one for each sub-shape/slice, s0..3--it may also require a shape
// tag indicating which of s0..3 we're traversing.
// ..
// The actual algorithm will:
// 1. determing which sub shape we're collecting in--this logic is needs the
//    intersection classification--simply checking the y-coords is enough.
// 2. If the y-coords of the current edge cross
//
//TODO: how to determine where to inject the contact points?
// - project all contact points onto current edge and select by smallest
//  distance? Is there an edge case this fails for?
//  - Let's try a sloppy first pass. If it bugs out, we can do a full contact
//  order pass before the sup-shape vertex collection pass, that can do the
//  distance test with all contacts for all edges, and then pick the best
//  overall matches of contact to edge.
// - If we can order the contact points within the Path, everything becomes
// easy. We can pick as we go with min(distance(c, edge)), and each time we
// cross a contact point, it's inserted in the correct order
//
//
//
//
//               v0       v1
//              ┌───────┐
//              │  s0   │
//          c_lu│       │c_ru
// ────────────►x ───── x◄────────────
//              │       │
//              │       │v2      v3
//              │  s1   └───────┐
//              │               │
//          c_lb│               │
// ────────────►x ───────────── x◄────
//              │     s2        │c_rb
//              └───────────────┘
//             v5                v4
//
//
//
//
fn test_collider_ray_cast(
    mut commands: Commands,
    mut collision_events: EventReader<CollisionEvent>,
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
    let ray_origin = Vec2::new(BRICK_DIM * 5.0, BRICK_DIM * -7.5);
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

            /*
            rapier_context.intersections_with_ray(
                ray_origin,
                ray_dir,
                max_toi,
                false, // solid?
                QueryFilter::only_dynamic(),
                |_: Entity, ray_intersection| {
                    draw_circle_contact(
                        ray_intersection.point,
                        commands.reborrow(),
                    );
                    draw_ray(
                        commands.reborrow(),
                        ray_origin,
                        ray_intersection.point,
                    );
                    true
                },
            );
            */

            let ray_origin_left =
                Vec2::new(BRICK_DIM * -5.0, BRICK_DIM * -6.5);
            let ray_dir_left = Vec2::new(1.0, 0.0);
            if let Some((ent, toi)) = rapier_context.cast_ray(
                ray_origin_left,
                ray_dir_left,
                max_toi,
                false,
                QueryFilter::only_dynamic(),
            ) {
                let hit = ray_origin_left + ray_dir_left * toi;
                //draw_circle_contact(hit, commands.reborrow());
                //draw_ray(commands.reborrow(), ray_origin_left, hit)
            }
            // top left ray (ray origin <----- contact)
            //rapier_context.intersections_with_ray(
            //    ray_origin_left,
            //    ray_dir_left,
            //    max_toi,
            //    false, // solid?
            //    QueryFilter::only_dynamic(),
            //    |_: Entity, ray_intersection| {
            //        draw_circle_contact(
            //            ray_intersection.point,
            //            commands.reborrow(),
            //        );
            //        draw_ray(
            //            commands.reborrow(),
            //            ray_origin_left,
            //            ray_intersection.point,
            //        );
            //        true
            //    },
            //);

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
            //
            // 3. convert from ActiveTetroid to TetroidDebris (which should
            //    be a side effect of despawning the entity and spawning
            //    the calculated fragments
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
        Stroke::new(Color::RED, 1.0),
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
        Stroke::new(Color::RED, 1.0),
    ));
}
