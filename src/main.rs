#![allow(unused, clippy::type_complexity, clippy::too_many_arguments)]
use std::{cmp::Ordering, collections::HashMap, f32::consts::PI};

use bevy::{prelude::*, transform, window::WindowResolution};
use bevy_prototype_lyon::prelude::*;
use bevy_rapier2d::{
    geometry::{shape_views::ConvexPolygonView, RayIntersection},
    parry::{
        query::details::contact_manifolds_trimesh_shape_shapes,
        shape::ConvexPolygon,
    },
    prelude::*,
    rapier::{dynamics::LockedAxes, geometry::ActiveCollisionTypes},
};

use bevy::gizmos::prelude::*;

use bevy_inspector_egui::quick::WorldInspectorPlugin;

mod arena;
mod event_demo;
mod tetroid;

use arena::{spawn_arena, Ground};
use event_demo::DebugShape;
use tetroid::components::*;
use tetroid::{spawn_lblock, BRICK_DIM};

// width/height of single square
const PIXELS_PER_METER: f32 = 50.0;

const IMPULSE_SCALAR: f32 = 20000.0;

fn main() {
    event_demo::app();
}

fn _main() {
    App::new()
        .add_event::<HitGround>()
        .add_event::<NextTetroid>()
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
        //.insert_resource(GizmoConfig {
        //    depth_bias: -1.0,
        //    ..Default::default()
        //})
        // Physics plugins
        .add_plugins(WorldInspectorPlugin::new())
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
        .add_systems(
            Update,
            (
                kbd_input,
                freeze_on_ground_contact_compound,
                cast_rays_compound,
                tetroid_spawner,
            )
                .chain(),
        )
        .run();
}

fn setup_graphics(mut commands: Commands) {
    // Add a camera so we can see the debug-render.
    commands.spawn(Camera2dBundle::default());
}

#[derive(Event)]
struct HitGround(Entity);

#[derive(Event)]
struct NextTetroid;

#[derive(Component)]
struct Row(u8);

#[derive(Component, Debug)]
struct Vertices {
    vs: Vec<Vec2>,
}

#[derive(Event, Debug)]
struct Pause;

fn kbd_input(
    kbd_input: Res<ButtonInput<KeyCode>>,
    mut ext_impulses: Query<&mut ExternalImpulse, With<ActiveTetroid>>,
    mut pause: EventWriter<Pause>,
    //active_tetroid_query: Query<Entity, With<ActiveTetroid>>,
) {
    if kbd_input.just_pressed(KeyCode::Escape) {
        pause.send(Pause);
    }

    let mut torque_impulse = 0.0;
    let torque_imp = 10.0 * IMPULSE_SCALAR;
    if kbd_input.pressed(KeyCode::KeyZ) {
        torque_impulse = torque_imp;
    } else if kbd_input.pressed(KeyCode::KeyX) {
        torque_impulse = -torque_imp;
    }

    let mut impulse = Vec2::new(0.0, 0.0);
    let lateral_imp = 5.0 * IMPULSE_SCALAR;
    let vertical_imp = lateral_imp;
    
    if kbd_input.pressed(KeyCode::ArrowLeft) {
        impulse.x = -lateral_imp;
    }
    if kbd_input.pressed(KeyCode::ArrowRight) {
        impulse.x = lateral_imp;
    }
    if kbd_input.pressed(KeyCode::ArrowDown) {
        impulse.y = -vertical_imp;
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
        *ext_impulse = ExternalImpulse {
            torque_impulse,
            impulse,
        };
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
        .insert(Row(0))
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
        .insert(Row(0))
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
        DebugShape
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

fn get_component_shapes(
    mut collider_query: Query<&mut Collider, With<ActiveTetroid>>,
    hit_ground_events: EventReader<HitGround>,
) {
    if hit_ground_events.is_empty() {
        return;
    }
    if let Some(compound) = collider_query
        .get_single_mut()
        .ok()
        .map(Mut::into_inner)
        .and_then(|c| c.as_compound())
    {
        for (pos, rot, collider_view) in compound.shapes() {
            println!("shape_type: {:?}", collider_view);
        }
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
        gizmos.linestrip(vs_global, Color::Srgba(Srgba::GREEN));
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
fn _test_collider_ray_cast() {}

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
        Stroke::new(Color::Srgba(Srgba::BLACK), 1.0),
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

// TODO: refactor as event: SliceEvent(Row)
//
// Cast rays for all rows to update density calculations and conditionally slice
// rows on tetroid deactivation.
//
// algorithm:
fn cast_rays_compound(
    //mut gizmos: Gizmos,
    mut cmds: Commands,
    rc: Res<RapierContext>,
    mut ev_hit_ground: EventReader<HitGround>,
    mut collider_query: Query<&GlobalTransform, With<ActiveTetroid>>,
    mut parent_at: Query<(Entity, &mut Children), With<ActiveTetroid>>,
    components: Query<&Collider, With<ActiveTetroidCollider>>,
    global_transforms: Query<&GlobalTransform, With<ActiveTetroidCollider>>,
    mut next_tetroid_writer: EventWriter<NextTetroid>,
) {
    for hit_ground in ev_hit_ground.read() {
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
        let filter = QueryFilter::new();

        // Cast rays -> classify (though this will *almost* always be a
        // Double-intersection).
        // left-upper

        //let bundle_hit = move |origin: Vec2, dir: Vec2| {
        //    move |(entity, toi): (Entity, f32)| (entity, origin + dir * toi)
        //};

        //let bundle_hit_ray_intersection =
        //    move |origin: Vec2, dir: Vec2| move |(ent, ri)| (ent, ri);

        let mut active_component_map: HashMap<Entity, Vec<Vec2>> =
            HashMap::new();
        let mut hits: Vec<(Entity, Vec2)> = Vec::new();
        let mut upper: Vec<Vec2> = Vec::new();
        let mut middle: Vec<Vec2> = Vec::new();
        let mut lower: Vec<Vec2> = Vec::new();

        // TODO: for each vertex and contact point for a given collider, add all
        // hits to one of upper, middle, or lower.
        //
        // FIXME:
        // - loop through children
        // - for each child: collect vertices and determine contact_points
        //
        if let Ok(global_transform) = collider_query.get_single_mut() {
            let mut on_hit = |entity: Entity, hit: RayIntersection| {
                //draw_circle_contact(ray_intersection.point, cmds.reborrow());
                //let global_contact_point = global_transform
                //.transform_point(Vec3::new(hit.point.x, hit.point.y, 0.0));
                let p = hit.point;
                hits.push((entity, p));
                match active_component_map.get_mut(&entity) {
                    None => {
                        active_component_map.insert(entity, vec![p]);
                    }
                    Some(v) => v.push(p),
                }

                true
            };

            rc.intersections_with_ray(
                lu_origin,
                l_dir,
                max_toi,
                true,
                filter,
                &mut on_hit,
            );

            rc.intersections_with_ray(
                lb_origin,
                l_dir,
                max_toi,
                false,
                filter,
                &mut on_hit,
            );
            rc.intersections_with_ray(
                ru_origin,
                r_dir,
                max_toi,
                false,
                filter,
                &mut on_hit,
            );
            rc.intersections_with_ray(
                rb_origin,
                r_dir,
                max_toi,
                false,
                filter,
                &mut on_hit,
            );

            for (entity, hit) in &hits {
                //draw_circle_contact(*hit, cmds.reborrow());
                //rc.intersection_with_ray(hit, l_dir, false, filter)
            }

            //if let Ok((at, mut children)) = parent_at.get_single_mut() {
            for (at, mut children) in parent_at.iter() {
                cmds.entity(at).remove::<ActiveTetroid>().insert(Debris);

                //children.into_iter().for_each(|e| {
                //    active_component_map.insert(e, Vec::new());
                //});

                // sort by child component id
                //hits.sort_by_key(|(e, _)| *e);
                //dbg!(&hits);
                //children.sort_by_key(|id| *id);
                for child in children.iter() {
                    cmds.entity(*child)
                        .remove::<ActiveTetroidCollider>()
                        .insert(DebrisCollider);

                    // we have the hits per component in `active_component_map`,
                    // now we just need to query the vertices
                    if let Some(col) = components
                        .get(*child)
                        .ok()
                        .and_then(Collider::as_convex_polygon)
                    {
                        let mut upper: Vec<Vec2> = Vec::new();
                        let mut middle: Vec<Vec2> = Vec::new();
                        let mut lower: Vec<Vec2> = Vec::new();

                        if let Some(v) = active_component_map.get_mut(child) {
                            let mut partition =
                                |p: &Vec2, high: f32, low: f32| {
                                    if p.y > high {
                                        upper.push(*p);
                                    } else if p.y == high {
                                        upper.push(*p);
                                        middle.push(*p);
                                    } else if p.y > low {
                                        middle.push(*p);
                                    } else if p.y == low {
                                        middle.push(*p);
                                        lower.push(*p);
                                    } else {
                                        lower.push(*p);
                                    }
                                };
                            let global_transform =
                                global_transforms.get(*child).unwrap();
                            col.points().for_each(|p| {
                                let global_p = v3_to_2(
                                    &global_transform
                                        .transform_point(v2_to_3(&p)),
                                );
                                partition(&global_p, lu_origin.y, lb_origin.y)
                            });
                            v.iter().for_each(|p| {
                                partition(p, lu_origin.y, lb_origin.y)
                            });

                            // NOTE: we're only concerned with child colliders subject
                            // to slicing, if a collider doesn't intersect any ray, it
                            // won't be drawn. That is, only new colliders are drawn,
                            // old ones are left as is
                            sort_convex_hull(&mut upper);
                            sort_convex_hull(&mut middle);
                            sort_convex_hull(&mut lower);
                            //println!(
                            //    "upper: {:?},\n middle: {:?}, lower: {:?}",
                            //    &upper, &middle, &lower
                            //);
                            draw_convex_hull(
                                cmds.reborrow(),
                                upper,
                                Color::Srgba(Srgba::RED),
                            );
                            draw_convex_hull(
                                cmds.reborrow(),
                                middle,
                                Color::Srgba(Srgba::GREEN),
                            );
                            draw_convex_hull(
                                cmds.reborrow(),
                                lower,
                                Color::Srgba(Srgba::BLUE),
                            );

                            // draw rays
                            draw_ray(cmds.reborrow(), ru_origin, lu_origin);
                            draw_ray(cmds.reborrow(), rb_origin, lb_origin);

                            //let c = a.chain(b);
                            //for p in col.points().chain(v.iter()) {
                            //    // place each point into its respective bucket
                            //}
                        }
                    }
                }

                //cmds.entity(at).despawn_recursive();
                //spawn_lblock(cmds.reborrow());
            }
        }

        //let mlu_hit = rc
        //    .cast_ray_and_get_normal(lu_origin, l_dir, max_toi, false, filter)
        //    .map(bundle_hit_ray_intersection(lu_origin, l_dir));
        //let mlb_hit = rc
        //    .cast_ray(lb_origin, l_dir, max_toi, true, filter)
        //    .map(bundle_hit(lb_origin, l_dir));
        //let mru_hit = rc
        //    .cast_ray(ru_origin, r_dir, max_toi, true, filter)
        //    .map(bundle_hit(ru_origin, r_dir));
        //let mrb_hit = rc
        //    .cast_ray(rb_origin, r_dir, max_toi, true, filter)
        //    .map(bundle_hit(rb_origin, r_dir));

        next_tetroid_writer.send(NextTetroid);
    }
}

/// On ground contact remove all velocity & gravity for ActiveTetroid, remove ActiveTetroid from entity
///
/// Event order is:
/// HitGround -> UpdateDensity -> Slice -> UnfreezeDebris
fn freeze_on_ground_contact_compound(
    mut commands: Commands,
    mut event_writer: EventWriter<HitGround>,
    mut collision_events: EventReader<CollisionEvent>,
    rc: Res<RapierContext>,
    mut ground_query: Query<Entity, (With<Collider>, With<Ground>)>,
    mut active_tetroid_query: Query<
        (Entity, &mut Velocity, &mut GravityScale, &mut Sleeping),
        (With<ActiveTetroid>),
    >,
) {
    if let Ok(ground) = ground_query.get_single_mut() {
        if let Ok((at, mut vel, mut gs, mut sleeping)) =
            active_tetroid_query.get_single_mut()
        {
            for ce in collision_events.read() {
                if let CollisionEvent::Started(collider, g, _) = ce {
                    //info!("collision: {:?}", (collider, g));
                    if let Some(parent) = rc.collider_parent(*collider) {
                        // collision with active tetroid
                        if parent == at && ground == *g {
                            info!("HitGround({:?})", parent);
                            event_writer.send(HitGround(parent));
                            *vel = Velocity::zero();
                            gs.0 = 0.0;
                            //sleeping.sleeping = true;

                            // locked: bool, wake_up: bool

                            //lock_translations(true, true)
                        }
                    }
                }
            }
        }
    }
}

/// Listens for HitGround or HitDebris and handles tetroid deactivation, spawns
/// new tetroid
fn tetroid_spawner(
    mut cmds: Commands,
    mut next_tetroid_reader: EventReader<NextTetroid>, // FIXME:
) {
    next_tetroid_reader.read().for_each(|_| {
        spawn_lblock(cmds.reborrow());
    })
}
