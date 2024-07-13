//! Event loop demo
//!
//! An attempt to sort out SpawnTetroid -> (HitDebris | HitGround) loop
//!
//! Tetroid spawns, falls and once it hits the ground or debris the `Freeze`
//! event is sent. Once all calculations are done, the tetroid is deactivated,
//! `Unfreeze` sends, the physics simulation unpauses, and `SpawnTetroid` is
//! sent.

use crate::{draw_circle_contact, draw_ray, Pause};
use bevy::input::common_conditions::input_just_pressed;
use bevy::utils::HashMap;
use bevy::{prelude::*, window::WindowResolution};
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_prototype_lyon::plugin::ShapePlugin;
use bevy_rapier2d::prelude::*;

use crate::arena::{self, Ground};
use crate::tetroid::{components::*, spawn_lblock, BRICK_DIM};
use crate::{
    cast_rays_compound, draw_convex_hull, kbd_input, sort_convex_hull,
    v2_to_3, v3_to_2,
};

#[derive(Event, Debug)]
struct Freeze;

#[derive(Event, Debug)]
struct UnFreeze;

#[derive(Event, Debug)]
struct DeactivateTetroid(Entity);

#[derive(Event, Debug)]
struct SpawnTetroid;

#[derive(Component, Debug)]
pub(crate) struct Tetroid;

const PIXELS_PER_METER: f32 = 50.0;
const GRAVITY: f32 = 0.05;
const VELOCITY: Velocity = Velocity {
    linvel: Vec2::new(0.0, -100.0),
    angvel: 0.0,
};

pub fn app() {
    App::new()
        .add_event::<Freeze>()
        .add_event::<UnFreeze>()
        .add_event::<DeactivateTetroid>()
        .add_event::<Pause>()
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
        //.add_systems(Startup, (spawn_block, arena::spawn_arena))
        .add_systems(Startup, (spawn_lblock, arena::spawn_arena))
        .add_systems(Update, (kbd_input, toggle_pause).chain())
        .add_systems(
            Update,
            (
                //tetroid_hit,
                tetroid_hit_compound,
                freeze,
                cast_rays,
                deactivate_tetroid,
                unfreeze, // TODO consolidate unfreeze and handle_unfreeze
                handle_unfreeze,
                block_spawner,
            )
                .chain(),
        )
        //        .add_systems(Startup, setup_physics)
        //.add_systems(Update, kbd_input)
        //.add_systems(Update, tetroid_spawner)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    // Add a camera so we can see the debug-render.
    commands.spawn(Camera2dBundle::default());
}

fn spawn_block(mut cmds: Commands) {
    let id = cmds
        .spawn(RigidBody::Dynamic)
        .insert(Collider::cuboid(BRICK_DIM, BRICK_DIM))
        .insert(ColliderMassProperties::Mass(1000.0))
        .insert(Sleeping::default())
        .insert(Restitution {
            coefficient: 0.0,
            combine_rule: CoefficientCombineRule::Min,
        })
        .insert(ActiveTetroid)
        .insert(Tetroid)
        .insert(TransformBundle::from(Transform::from_xyz(
            -BRICK_DIM, 320.0, 0.0,
        )))
        .insert(ExternalImpulse {
            impulse: Vec2::new(0.0, 0.0),
            torque_impulse: 0.0,
        })
        .insert(ActiveEvents::COLLISION_EVENTS)
        .insert(Ccd::enabled()) // enable continous collision detection
        .insert(VELOCITY)
        .insert(GravityScale(GRAVITY))
        .id();
    info!("spawn_block: {:?}", id);
}

/// Send `Freeze` event if active tetroid hits ground or other tetroid.
fn tetroid_hit_compound(
    mut freezes: EventWriter<Freeze>,
    mut collisions: EventReader<CollisionEvent>,
    children: Query<&Children>,
    active_tetroid: Query<Entity, With<ActiveTetroid>>,
    ground: Query<Entity, With<Ground>>,
    tetroids: Query<Entity, With<Tetroid>>,
    rc: Res<RapierContext>,
    //wall: Query<Entity, With<arena::Wall>>,
) {
    // FIXME why is this triggering for collisions with the wall
    let ground = ground.single();
    // for each collider of the active tetroid check contactt
    if let Ok(at) = active_tetroid.get_single() {
        for child in children.iter_descendants(at) {
            for contact in rc.contact_pairs_with(child) {
                if contact.has_any_active_contacts() {
                    // determine which collider handle is `child`
                    let other = if contact.collider1() == child {
                        contact.collider2()
                    } else {
                        contact.collider1()
                    };
                    // check if `other` is `ground`
                    if tetroids.contains(other) || other == ground {
                        //dbg!(at, child, other);
                        info!("Freeze");
                        freezes.send(Freeze);
                    }
                }
            }
        }
    }
}

//
fn tetroid_hit(
    mut freezes: EventWriter<Freeze>,
    mut collisions: EventReader<CollisionEvent>,
    active_tetroid: Query<Entity, With<ActiveTetroid>>,
    ground: Query<Entity, With<Ground>>,
    tetroids: Query<Entity, With<Tetroid>>,
    rc: Res<RapierContext>,
) {
    if let Ok(at) = active_tetroid.get_single() {
        let ground = ground.single();
        for e in collisions.read() {
            // if active tetroid hits any TetroidStuff or Ground
            if let CollisionEvent::Started(a, b, _) = e {
                //if let Some(parent) = rc.collider_parent(*a) { }
                // - determine if one collider is the ground or a tetroid
                // - determine if (and then which) collider is a child of the
                //   active tetroid

                let mut other = *a;
                if at == *a {
                    other = *b;
                } else if at == *b {
                    other = *a;
                } else {
                    return;
                }
                if other == ground {
                    info!("hit ground: Freeze: {:?}, {:?}", a, b);
                    freezes.send(Freeze);
                } else if tetroids.contains(other) {
                    info!(
                        "hit tetroid stuff/debris: Freeze: {:?}, {:?}",
                        a, b
                    );
                    freezes.send(Freeze);
                }
            }
        }
    }
}

fn freeze(
    mut cmds: Commands,
    mut tetroids: Query<
        (Entity, &mut Velocity, &mut GravityScale, &mut Sleeping),
        With<Tetroid>,
    >,
    mut freeze: EventReader<Freeze>,
) {
    if !freeze.is_empty() {
        freeze.clear();
        //dbg!("in freeze, {}", tetroids.iter().len());
        for (entity, mut vel, mut gravity, mut sleeping) in tetroids.iter_mut()
        {
            //sleeping.sleeping = true;

            //info!("Freezing all tetroids, id: {:?}, {:?}", entity, sleeping);
            *vel = Velocity::zero();
            gravity.0 = 0.0;
        }
    }
}

fn handle_unfreeze(
    mut cmds: Commands,
    mut tetroids: Query<
        (Entity, &mut Velocity, &mut GravityScale, &mut Sleeping),
        With<Tetroid>,
    >,
    mut unfreeze: EventReader<UnFreeze>,
) {
    if !unfreeze.is_empty() {
        unfreeze.clear();
        //dbg!("in freeze, {}", tetroids.iter().len());
        for (entity, mut vel, mut gravity, mut sleeping) in tetroids.iter_mut()
        {
            //sleeping.sleeping = true;

            //info!("Freezing all tetroids, id: {:?}, {:?}", entity, sleeping);
            *vel = VELOCITY;
            gravity.0 = 1.0;
        }
    }
}
fn deactivate_tetroid(
    mut cmds: Commands,
    mut active_tetroid: Query<(Entity, &Children), With<ActiveTetroid>>,
    mut deactivate: EventWriter<DeactivateTetroid>,
    mut freeze: EventReader<Freeze>,
) {
    if !freeze.is_empty() {
        if let Ok((at, children)) = active_tetroid.get_single_mut() {
            cmds.entity(at).remove::<ActiveTetroid>();
            for &child in children {
                cmds.entity(child).remove::<ActiveTetroidCollider>();
            }

            info!("DeactivateTetroid({:?})", at);
            deactivate.send(DeactivateTetroid(at));
        }
        freeze.clear();
    }
}

fn unfreeze(
    mut deactivate: EventReader<DeactivateTetroid>,
    mut freeze: EventReader<Freeze>,
    mut unfreeze: EventWriter<UnFreeze>,
) {
    if !freeze.is_empty() && !deactivate.is_empty() {
        unfreeze.send(UnFreeze);
        info!("Unfreeze");
        freeze.clear();
        deactivate.clear();
    }
}

fn block_spawner(mut cmds: Commands, mut freeze: EventReader<Freeze>) {
    if !freeze.is_empty() {
        freeze.clear();
        spawn_lblock(cmds.reborrow());
    }
}

// TODO:
// [ ] - ignores vertices on wall (at origin) during raycast
// [ ] - only triggers on hit ground not hit tetroid
fn cast_rays(
    //mut gizmos: Gizmos,
    mut cmds: Commands,
    rc: Res<RapierContext>,
    mut freeze: EventReader<Freeze>,
    mut collider_query: Query<&GlobalTransform, With<ActiveTetroid>>,
    mut parent_at: Query<(Entity, &mut Children), With<ActiveTetroid>>,
    components: Query<&Collider, With<ActiveTetroidCollider>>,
    global_transforms: Query<&GlobalTransform, With<ActiveTetroidCollider>>,
    //mut next_tetroid_writer: EventWriter<NextTetroid>,
) {
    if !freeze.is_empty() {
        freeze.clear();
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
            true,
            filter,
            &mut on_hit,
        );
        rc.intersections_with_ray(
            ru_origin,
            r_dir,
            max_toi,
            true,
            filter,
            &mut on_hit,
        );
        rc.intersections_with_ray(
            rb_origin,
            r_dir,
            max_toi,
            true,
            filter,
            &mut on_hit,
        );

        //if let Ok((at, mut children)) = parent_at.get_single_mut() {
        for (at, mut children) in parent_at.iter() {
            draw_circle_contact(lu_origin, cmds.reborrow());
            warn!("here");
            //cmds.entity(at).remove::<ActiveTetroid>().insert(Debris);

            //children.into_iter().for_each(|e| {
            //    active_component_map.insert(e, Vec::new());
            //});

            // sort by child component id
            //hits.sort_by_key(|(e, _)| *e);
            //dbg!(&hits);
            //children.sort_by_key(|id| *id);
            for child in children.iter() {
                //cmds.entity(*child)
                //    .remove::<ActiveTetroidCollider>()
                //    .insert(DebrisCollider);

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
                        let mut partition = |p: &Vec2, high: f32, low: f32| {
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
                                &global_transform.transform_point(v2_to_3(&p)),
                            );
                            partition(&global_p, lu_origin.y, lb_origin.y)
                        });
                        v.iter().for_each(|p| {
                            partition(p, lu_origin.y, lb_origin.y);
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
                        draw_convex_hull(cmds.reborrow(), upper, Color::RED);
                        draw_convex_hull(
                            cmds.reborrow(),
                            middle,
                            Color::GREEN,
                        );
                        draw_convex_hull(cmds.reborrow(), lower, Color::BLUE);

                        // draw rays
                        draw_ray(cmds.reborrow(), ru_origin, lu_origin);
                        draw_ray(cmds.reborrow(), rb_origin, lb_origin);
                        info!("drawing partitions");

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

    //next_tetroid_writer.send(NextTetroid);
}

fn toggle_pause(
    mut time: ResMut<Time<Virtual>>,
    mut pause: EventReader<Pause>,
) {
    for p in pause.read() {
        if time.is_paused() {
            time.unpause();
        } else {
            time.pause();
        }
    }
}
