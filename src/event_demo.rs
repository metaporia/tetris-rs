//! Event loop deme
//!
//! An attempt to sort out SpawnTetroid -> (HitDebris | HitGround) loop
//!
//! Tetroid spawns, falls and once it hits the ground or debris the `Freeze`
//! event is sent. Once all calculations are done, the tetroid is deactivated,
//! `Unfreeze` sends, the physics simulation unpauses, and `SpawnTetroid` is
//! sent.

use bevy::asset::transformer;
use bevy::input::common_conditions::input_just_pressed;
use bevy::log::{Level, LogPlugin};
use bevy::transform::components::Transform;
use bevy::utils::HashMap;
use bevy::{prelude::*, window::WindowResolution};
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_prototype_lyon::{entity::Path, plugin::ShapePlugin};
use bevy_rapier2d::parry::transformation;
use bevy_rapier2d::prelude::*;

use itertools::Itertools;
use std::cmp::{Ord, Ordering};
use std::iter::{self, once, zip};
use std::time::Duration;

use crate::arena::{
    self, render_row_density, spawn_density_indicator_column,
    ClearRowDensities, Ground, RowDensity, RowDensityIndicatorMap, SliceReady,
};
use crate::arena::{check_row_densities, clear_row_densities};
use crate::image_demo::{
    apply_slice_image, image_handle_to_sprite_bundle, image_to_sprite_bundle,
    new_blue_square_bundle, rgba_image_to_sprite_bundle, ClearBelow,
    SliceImage, SquareImage, TetrominoAssetMap, TetrominoAssetPlugin,
};
use crate::tetroid::{
    components::*, spawn_tetromino, Tetromino, TetrominoBundle,
    TetrominoCollider, TetrominoColliderBundle, BRICK_DIM,
};
use crate::{
    draw_convex_hull, get_pause_input, graphics, kbd_input, physics,
    sort_convex_hull, v2_to_3, v3_to_2, window,
};
use crate::{draw_ray, image_demo, Pause};

use tetris_rs::{FreezePlugin, GameState};

#[derive(Event, Debug)]
pub struct Freeze;

#[derive(Event, Debug)]
pub struct UnFreeze;

#[derive(Event, Debug)]
pub struct DeactivateTetromino;

#[derive(Event, Debug)]
pub struct SpawnNextTetromino;

/// Marks component as being either a `Tetromino` or `TetroidCollider`.
///
/// "Tetroid" is used in the sense of being tetromino-ish.
#[derive(Component, Debug, Default)]
pub struct Tetroid;

/// Signal that the active tetromino has hit either the ground or another
/// tetromino
#[derive(Event, Debug)]
pub struct ActiveTetrominoHit;

pub const PIXELS_PER_METER: f32 = 50.0;
pub const GRAVITY: f32 = 0.05;
pub const VELOCITY: Velocity = Velocity {
    linvel: Vec2::new(0.0, -105.0),
    angvel: 0.0,
};
pub const FRICTION: f32 = 0.3;

pub const GROUND_Y: f32 = -BRICK_DIM * 18.0 / 2.0;

pub const INITIAL_FALLSPEED: f32 = -105.0;

#[derive(Resource)]
pub struct FallSpeed(f32);

impl FallSpeed {
    pub fn as_velocity(&self) -> Velocity {
        Velocity {
            angvel: 0.0,
            linvel: Vec2::new(0.0, self.0),
        }
    }
}

pub fn app() {
    let mut app = App::new();
    app.add_event::<Freeze>()
        .add_event::<ActiveTetrominoHit>()
        .add_event::<UnFreeze>()
        .add_event::<DeactivateTetromino>()
        .add_event::<Pause>()
        .add_event::<RowDensity>()
        //.add_event::<SliceRow>()
        .add_event::<SliceRows>()
        .add_event::<SpawnNextTetromino>()
        .add_event::<DespawnTetromino>()
        .add_event::<ClearRowDensities>()
        .add_event::<SliceImage>()
        .add_event::<SliceReady>()
        .insert_resource(HitMap::default())
        .insert_resource(Partitions::default())
        .insert_resource(RowDensityIndicatorMap::default())
        .insert_resource(FallSpeed(INITIAL_FALLSPEED))
        // Schedule graph
        //.add_plugins(bevy_mod_debugdump::CommandLineArgs)
        // Physics plugins
        //.add_plugins(RapierDebugRenderPlugin::default())
        // ##############
        // # MY PLUGINS #
        // ##############
        .add_plugins((window::plugin, physics::plugin, graphics::plugin))
        .add_plugins(FreezePlugin)
        .add_plugins(WorldInspectorPlugin::new())
        .observe(clear_row_densities)
        .observe(apply_slices)
        //.observe(image_demo::clear_below)
        .observe(rdeactivate_tetromino)
        //.observe(dbg_slice_rows)
        .init_state::<GameState>()
        .add_systems(
            Startup,
            (
                spawn_tetromino,
                arena::spawn_arena,
                spawn_density_indicator_column,
            ),
        )
        .add_systems(
            Update,
            (
                kbd_input.run_if(in_state(GameState::Playing)),
                get_pause_input,
                toggle_pause,
                reset_game.run_if(input_just_pressed(KeyCode::KeyR)),
                reset_tetroids.run_if(input_just_pressed(KeyCode::KeyT)),
                reset_debug_shapes.run_if(input_just_pressed(KeyCode::KeyC)),
            ),
        )
        .add_systems(OnEnter(GameState::Frozen), handle_freeze)
        .add_systems(OnExit(GameState::Frozen), handle_unfreeze)
        .add_systems(
            Update,
            (
                (
                    // TODO consolidate unfreeze and handle_unfreeze
                    // - we want a timer for `Freeze`
                    //unfreeze, // unfreeze if frozen from last frame
                    tetroid_hit_compound,
                    handle_active_tetromino_hit,
                    //handle_freeze,
                    (
                        // these two run every frame, uncoditionally
                        row_intersections,
                        partitions,
                        check_row_densities,
                    )
                        .chain(),
                    //deactivate_tetromino,
                    //handle_unfreeze,
                    block_spawner,
                )
                    .chain(),
                despawn_tetrominos,
                render_row_density,
                apply_slice_image,
            ),
        );

    //bevy_mod_debugdump::print_schedule_graph(&mut app, Update);

    app.run();
}

// END freeze

pub fn setup_graphics(mut commands: Commands) {
    // Add a camera so we can see the debug-render.
    commands.spawn(Camera2dBundle::default());
}

/// Send `Freeze` event if active tetroid hits ground or other tetroid.
fn tetroid_hit_compound(
    mut freezes: EventWriter<Freeze>,
    mut active_tetromino_hit: EventWriter<ActiveTetrominoHit>,
    children: Query<&Children>,
    active_tetroid: Query<Entity, With<ActiveTetromino>>,
    ground: Query<Entity, With<Ground>>,
    tetroids: Query<Entity, With<Tetroid>>,
    rc: Res<RapierContext>,
    //wall: Query<Entity, With<arena::Wall>>,
) {
    // FIXME why is this triggering for collisions with the wall
    let ground = ground.single();
    // for each collider of the active tetroid check contactt
    if let Ok(at) = active_tetroid.get_single() {
        let any_hits = children.iter_descendants(at).any(|child| {
            rc.contact_pairs_with(child).any(|contact| {
                if contact.has_any_active_contact() {
                    // determine which collider handle is `child`
                    let other = if contact.collider1() == child {
                        contact.collider2()
                    } else {
                        contact.collider1()
                    };
                    // check if `other` is `ground`
                    tetroids.contains(other) || other == ground
                } else {
                    false
                }
            })
        });
        if any_hits {
            info!("ActiveTetrominoHit");
            active_tetromino_hit.send(ActiveTetrominoHit);
            //info!("Freeze");
            //freezes.send(Freeze);
        }
    }
}

/// On ground contact remove all velocity & gravity for ActiveTetroid, remove
/// ActiveTetroid from entity
fn handle_freeze(
    mut tetroids: Query<
        (
            Entity,
            &mut Velocity,
            &mut GravityScale,
            &mut Sleeping,
            &mut ExternalForce,
            &mut Damping,
        ),
        With<Tetroid>,
    >,
    mut freeze: EventReader<Freeze>,
) {
    //if !freeze.is_empty() {
    //freeze.clear();
    if true {
        //dbg!("in freeze, {}", tetroids.iter().len());
        info!("Freezing");
        for (
            entity,
            mut vel,
            mut gravity,
            mut sleeping,
            mut ext_force,
            mut damping,
        ) in tetroids.iter_mut()
        {
            sleeping.sleeping = true;
            damping.linear_damping = 0.0;
            ext_force.force = Vec2::ZERO;

            //info!("Freezing all tetroids, id: {:?}, {:?}", entity, sleeping);
            *vel = Velocity::zero();
            gravity.0 = 0.0;
        }
    }
}

/// TODO: unfreeze with some momentum
fn handle_unfreeze(
    mut tetroids: Query<
        (
            Entity,
            &mut Velocity,
            &mut GravityScale,
            &mut ExternalForce,
            &mut Damping,
            &mut Sleeping,
        ),
        With<Tetroid>,
    >,
    mut unfreeze: EventReader<UnFreeze>,
    mut fallspeed: Res<FallSpeed>,
) {
    if true {
        //}!unfreeze.is_empty() {
        //unfreeze.clear();
        //dbg!("in freeze, {}", tetroids.iter().len());
        //info!("Unfreezing");
        for (
            entity,
            mut vel,
            mut gravity,
            mut external_force,
            mut damping,
            sleeping,
        ) in tetroids.iter_mut()
        {
            //sleeping.sleeping = true;

            //info!("Freezing all tetroids, id: {:?}, {:?}", entity, sleeping);
            *vel = fallspeed.as_velocity();
            gravity.0 = 1.0;
            external_force.force = Vec2::ZERO;
            external_force.torque = 0.0;
            damping.linear_damping = 0.0;
        }
    }
}

// Ok. So event readers can't be accessed from an observer.
fn dbg_slice_rows(
    trigger: Trigger<SliceRows>,
    hits: EventReader<ActiveTetrominoHit>,
) {
    if !hits.is_empty() {
        error!("unique: found both events");
        dbg!(&trigger.event());
    }
}

/// Sends `DeactivateTetromino` and `SpawnNextTetroid` on `ActiveTetrominoHit`
/// event.
///
/// When it receive a hit event, it checks for any slices rows. If there are
/// any, the system is frozen for slicing.
fn handle_active_tetromino_hit(
    mut commands: Commands,
    mut active_tetromino_hit: EventReader<ActiveTetrominoHit>,
    mut freeze: EventWriter<Freeze>,
    //mut slices: EventReader<SliceReady>,
    mut next_tetromino: EventWriter<SpawnNextTetromino>,
) {
    if !active_tetromino_hit.is_empty() {
        active_tetromino_hit.clear();
        // triggers `rdeactivate_tetromino` observer
        commands.trigger(DeactivateTetromino);
        next_tetromino.send(SpawnNextTetromino);
        // TODO: check for row slices and freeze
        // - Since SliceRows is a triggered event, idk that we can read it with
        //   `EventReader<SliceRows>`
        //if !slices.is_empty() {
        //    error!("SliceReady inner");
        //    slices.clear();
        //    error!("Received ActiveTetrominoHit & SliceRows: Freezing");
        //    freeze.send(Freeze);
        //}
    }
}

// Removes `Active*` marker components from tetromino. Removes damping, forces
// from body and applies (higher) gravity scale for inactive bodies.
//
fn rdeactivate_tetromino(
    trigger: Trigger<DeactivateTetromino>,
    mut cmds: Commands,
    mut active_tetroid: Query<
        (
            Entity,
            &mut Velocity,
            &mut GravityScale,
            &mut ExternalForce,
            &mut Damping,
            &Children,
        ),
        With<ActiveTetromino>,
    >,
) {
    if let Ok((
        at,
        mut velocity,
        mut gravity,
        mut ext_force,
        mut damping,
        children,
    )) = active_tetroid.get_single_mut()
    {
        velocity.linvel.y = -120.0;
        gravity.0 = 2.0;
        ext_force.force = Vec2::ZERO;
        ext_force.torque = 0.0;
        damping.linear_damping = 0.0;

        info!("Deactivating Tetromino: {:?}", &at);
        cmds.entity(at).remove::<ActiveTetromino>();
        for &child in children {
            cmds.entity(child).remove::<ActiveTetrominoCollider>();
        }
    }
}

// Check if active tetroid needs to be deactivated (has ActiveTetroidHit fired)
fn deactivate_tetromino(
    mut cmds: Commands,
    mut active_tetroid: Query<(Entity, &Children), With<ActiveTetromino>>,
    mut deactivate: EventWriter<DeactivateTetromino>,
    mut freeze: EventReader<Freeze>,
) {
    if !freeze.is_empty() {
        if let Ok((at, children)) = active_tetroid.get_single_mut() {
            cmds.entity(at).remove::<ActiveTetromino>();
            for &child in children {
                cmds.entity(child).remove::<ActiveTetrominoCollider>();
            }

            info!("DeactivateTetroid({:?})", at);
            deactivate.send(DeactivateTetromino);
        }
        freeze.clear();
    }
}

fn unfreeze(
    //mut deactivate: EventReader<DeactivateTetromino>,
    mut freeze: EventReader<Freeze>,
    mut unfreeze: EventWriter<UnFreeze>,
) {
    if !freeze.is_empty() {
        // && !deactivate.is_empty() {
        freeze.clear();
        info!("Unfreeze");
        unfreeze.send(UnFreeze);
        //deactivate.clear();
    }
}

fn block_spawner(
    mut cmds: Commands,
    mut freeze: EventReader<Freeze>,
    mut next_tetroid: EventReader<SpawnNextTetromino>,
    mut images: ResMut<Assets<Image>>,
    fallspeed: Res<FallSpeed>,
    asset_map: Res<TetrominoAssetMap>,
) {
    //if !freeze.is_empty() || !next_tetroid.is_empty() {
    if !next_tetroid.is_empty() {
        //freeze.clear();
        next_tetroid.clear();
        spawn_tetromino(cmds.reborrow(), images, fallspeed, asset_map);
    }
}

type Ray = u8;

/// An ephemeral `Resource` mapping `(Entity, Ray)` tuples to a the list of ray
/// intersections with that entity, for Ray in 1..ROWS.
#[derive(Resource, Debug, Default)]
struct HitMap(HashMap<(Entity, Ray), Vec<Vec2>>);

impl HitMap {
    /// A collider is "in" a `Row`, r, if it has entries
    /// for `Ray`s r or r+1 (if they exist)
    ///
    /// NOTE: fuck this doesn't catch colliders fully inside the row but its
    /// fine so long as the despawn pass catches em. For the purposes of
    /// creating the convex hulls and populating the partition map, this is
    /// FIXME: this ignore
    fn colliders_in_row(&self, row: Row) -> impl Iterator<Item = &Entity> {
        self.0
            .keys()
            .filter_map(|(col, ray)| {
                if row == *ray || row + 1 == *ray {
                    Some(col)
                } else {
                    None
                }
            })
            .sorted()
            .dedup()
    }
}

/// Returns co
fn children_to_unique_parents(
    colliders: impl Iterator<Item = Entity>,
) -> Vec<Entity> {
    colliders.sorted().dedup().collect()
}

/// Return colliders in a given row. This is exhaustive and includes colliders
/// not hit by rays.
///
fn colliders_in_row<'a>(
    row: Row,
    colliders: impl Iterator<Item = (Entity, &'a Collider, &'a GlobalTransform)>,
) -> Vec<(Entity, Vec<Vec2>)> {
    colliders
        .into_iter()
        .filter_map(|(id, c, t)| {
            if let Some(view) = c.as_convex_polygon() {
                let ps: Vec<Vec2> =
                    view.points().map(|p| transform_point(t)(&p)).collect();
                if ps.iter().any(is_point_in_row(row)) {
                    let t = (id, ps);
                    //cs.push(t)
                    Some(t)
                } else {
                    None
                }
            } else {
                None
            }
        })
        .collect()
}

struct ColliderData {
    id: Entity,
    image_handle: Handle<Image>,
    transform: Transform,
    global_transform: GlobalTransform,
}

/// Apply slices per `Tetromino`.
///
/// This is an observer so it should run immediately after the calling system's
/// command queue is flushed (so when` partitions` finishes).
fn apply_slices(
    trigger: Trigger<SliceRows>,
    mut cmds: Commands,
    mut despawn: EventWriter<DespawnTetromino>,
    mut partitions: ResMut<Partitions>,
    children: Query<&Children, With<Tetromino>>,
    tetrominos: Query<Entity, With<Tetromino>>,
    transforms: Query<(&Transform, &GlobalTransform), With<TetrominoCollider>>,
    tetroid_colliders: Query<
        (Entity, &Collider, &GlobalTransform),
        With<TetrominoCollider>,
    >,
    mut images: ResMut<Assets<Image>>,
    image_handles: Query<&Handle<Image>, With<SquareImage>>,
    collider_children: Query<&Children, With<TetrominoCollider>>,
    mut slice_images: EventWriter<SliceImage>,
) {
    // TODO:
    // [x] get_rows_of
    // [x] get children for each parent
    // [x] for child:
    //     if child in rows -> push remaining partitions
    //     otherwise -> push transformed collider points
    // [x] group_hulls(hulls)
    // [x] send cleanup events

    dbg!("{}", partitions.0.len());

    let slice_rows = trigger.event();
    dbg!(&slice_rows);

    dbg!(tetrominos.iter().len());
    for t in tetrominos.iter() {
        let child_data = get_rows_of(t, &children, &tetroid_colliders);
        //dbg!(&child_data);

        let child_bounds =
            child_data.iter().flat_map(|(_, (lower, upper), _)| {
                iter::once(lower.row).chain(iter::once(upper.row))
            });
        //let bs: Vec<u8> = child_data
        //    .iter()
        //    .flat_map(|(_, (lower, upper), _)| vec![lower.row, upper.row]).collect();
        //dbg!(child_data.len());
        //dbg!(&bs);
        let Some((t_lower, t_upper)) = get_min_max(child_bounds) else {
            break;
        };

        //dbg!(t_lower, t_upper);
        // determine if parent is in row, otherwise short-circuit
        let slice_parent = slice_rows
            .rows
            .iter()
            .any(|&r| r >= t_lower && r <= t_upper);

        // parent is in row, so:
        // - apply `SlicesRows` to each child
        // - send despawn event to t
        // AND
        // - remove child's `Handle<Image>`
        //   - spawn new spritebundle from extracted image
        //   - send `SliceImage { sprite_bundle_id: Entity, rows_to_slice: Vec<u8>}`
        //     --this will handle discarding pixels
        //   - push `(sprite_bundle_id, hull)` to hulls
        if slice_parent {
            //dbg!(slice_parent, t);
            // untouched child vertices and sliced vertices.
            // each element is (Collider handle, image handle, retained vertices)
            let mut hulls: Vec<(ColliderData, Vec<Vec2>)> = Vec::new();
            for (child, (lower, upper), vertices) in child_data {
                // get childs transform
                let Ok((transform, global_transform)) = transforms.get(child)
                else {
                    break;
                };
                // get old image handle
                // 1. get single child of collider, namely, it's sprite_bundle
                let Some(sprite) =
                    collider_children.iter_descendants(child).next()
                else {
                    error!(
                        "Found TetroidCollider, {:?}, without sprite",
                        &child
                    );
                    break;
                };
                // 2. get `Handle<Image> component from sprite bundle
                let Ok(image_handle) = image_handles.get(sprite) else {
                    error!(
                        "Found SpriteBundle, {:?}, without image handle",
                        &sprite
                    );
                    break;
                };

                let Some(image) = images.remove(image_handle) else {
                    error!("No image associated with image handle of sprite: {:?}", &sprite);
                    break;
                };

                let rows_to_slice: Vec<&u8> =
                    slice_rows.rows_within(&lower, &upper).collect();
                //dbg!(&child, &rows_to_slice);
                // child not to be sliced, spawn vertices as is
                if rows_to_slice.is_empty() {
                    // TODO: count children and remove image from Assets<Image> to
                    // avoid unnecessary cloning.
                    //
                    // TODO: in this case we could just remove the sprite
                    // bundle from it's old parent and pass along its id
                    // (that is, `sprite`).
                    // Then only below would we have to actually remove the
                    // image handle
                    //
                    // copy old image, no image slicing required as the
                    // collider was not in a slice row
                    //let sprite_bundle =
                    //    image_to_sprite_bundle(image.clone(), images.as_mut());
                    //let id = cmds.spawn(sprite_bundle).id();
                    let image_handle = images.add(image.clone());

                    let collider_data = ColliderData {
                        id: child,
                        image_handle,
                        transform: *transform,
                        global_transform: *global_transform,
                    };
                    //info!("child not in row, pushing as is");
                    hulls.push((
                        collider_data,
                        vertices, //.into_iter()
                                  //.map(|p| transform_point_to_local(transform, &p))
                                  //.collect(),
                    ));
                }
                // otherwise, child has applicable slices, apply 'em
                else {
                    // - get child's partitions

                    //dbg!(&partitions.0);
                    //dbg!(&child);
                    if let Some(parts) = partitions.0.get(&child) {
                        //info!("child has applicable slices");
                        for (row, part) in parts {
                            //dbg!(row, &rows_to_slice);
                            // push part if it doesn't have applicable slices
                            if !rows_to_slice.contains(&row) {
                                // FIXME: will doing this inside the for-lop
                                // over `parts` duplicate it? I'm not 100% sure
                                // that each hull is unique in `Partitions`
                                //let sprite_bundle = image_to_sprite_bundle(
                                //    image.clone(),
                                //    images.as_mut(),
                                //);
                                //let sprite_id = cmds.spawn(sprite_bundle).id();
                                let image_handle = images.add(image.clone());

                                //slice_images.send(SliceImage {
                                //    image_handle,
                                //    rows_to_slice: rows_to_slice
                                //        .clone()
                                //        .into_iter()
                                //        .cloned()
                                //        .collect(),
                                //});

                                // TODO: we should be able to just remove the
                                // partition entry to avoid the clone
                                //
                                let collider_data = ColliderData {
                                    id: child,
                                    image_handle,
                                    transform: *transform,
                                    global_transform: *global_transform,
                                };
                                hulls.push((
                                    collider_data,
                                    part.clone(), //iter() .cloned() .map(|p| { transform_point_to_local( transform, &p,) }) .collect(),
                                ));
                                //info!("pushing part");
                            }
                            // partition is sliced, so discard it. Likewise
                            // for its sprite bundle
                            //info!("skipping, {}", row);
                        }
                    }
                    //error!("child has applicable slices but partition entry not found");
                }
            }

            // With `spawn_hull_groups` working, can we not just collect
            // all the new hulls into a single `Vec` and pass it to
            // `spawn_hull_groups`
            //
            // This shit works.
            spawn_hull_groups(
                cmds.reborrow(),
                hulls,
                images.as_mut(),
                &mut slice_images,
                slice_rows,
            );

            // despawn parent tetromino
            if let Some(mut parent_cmds) = cmds.get_entity(t) {
                //info!("Despawing Tetromino: parent id = {:?}", &parent);
                //parent_cmds.despawn_descendants();
                //parent_cmds.despawn()
                info!("Event: DespawnTetromino({:?})", &t);
                despawn.send(DespawnTetromino(t));
            } else {
                warn!("No Tetromino parent to despawn: id = {:?}", t);
            };
        }
    }

    // done slicing, so wipe row densities
    cmds.trigger(ClearRowDensities);
}

/// Get the rows a `Tetromino`s child colliders intersect and each colliders
/// vertices, transformed into global space.
///
/// Takes a `Tetromino`s id and the necessary children queries.
///
/// NB: it's unclear whether there are downsides to passing queries to
/// non-systems.
fn get_rows_of(
    tetromino_id: Entity,
    children: &Query<&Children, With<Tetromino>>,
    tetroid_colliders: &Query<
        (Entity, &Collider, &GlobalTransform),
        With<TetrominoCollider>,
    >,
) -> Vec<(Entity, (RowBounds, RowBounds), Vec<Vec2>)> {
    let get_min_max_row_bounds = |cs: &Vec<Vec2>| {
        get_min_max(cs.iter().map(|p| p.y)).and_then(|(min, max)| {
            // calculate intersecting rows from bounds
            RowBounds::from_y(max).and_then(|upper| {
                RowBounds::from_y(min).map(|lower| (lower, upper))
            })
        })
    };

    // get child colliders and global transforms
    // kind of dense but:
    // - get collider vertices in local space
    // - apply transform
    // - get higest and lowest y-coordinates
    //let colliders: Vec<(Entity, (RowBounds, RowBounds), Vec<Vec2>)> =
    children
        .iter_descendants(tetromino_id)
        .filter_map(|child| {
            tetroid_colliders.get(child).ok().and_then(|(_, c, t)| {
                c.as_convex_polygon().and_then(|view| {
                    let ps = view
                        .points()
                        .map(|p| transform_point(t)(&p))
                        .collect::<Vec<Vec2>>();
                    get_min_max_row_bounds(&ps)
                        .map(|bounds| (child, bounds, ps))
                })
            })
        })
        .collect()
}

/// Takes a `GlobalTransform` and returns a closuer that applies the transform
/// to a `Vec2` with the necessary conversions to and from `Vec3`.
pub fn transform_point(
    global_transform: &GlobalTransform,
) -> impl FnMut(&Vec2) -> Vec2 + '_ {
    |p: &Vec2| global_transform.transform_point(p.extend(0.0)).xy()
}

/// Apply a `Transform` to a point in global space, converting it into the
/// local space of the transform.
fn transform_point_to_local(transform: &Transform, p: &Vec2) -> Vec2 {
    transform.transform_point(p.extend(0.0)).xy()
}

fn transform_hull<'a, 'b>(
    global_transform: &'a GlobalTransform,
    hull: &'b ConvexHull,
) -> impl Iterator<Item = Vec2> + 'b
where
    'a: 'b,
{
    hull.iter().map(transform_point(global_transform))
}

/// Get the highest and lowest of list of floats.
///
/// In success case returns `Some(min, max)`
pub fn get_min_max<T>(ys: impl Iterator<Item = T>) -> Option<(T, T)>
where
    T: PartialOrd + Copy,
{
    let f = |(min, max): (T, T), y| {
        if y < min {
            (y, max)
        } else if y > max {
            (min, y)
        } else {
            (min, max)
        }
    };
    let mut iter = ys.into_iter();
    iter.next().map(|first| iter.fold((first, first), f))
}

pub fn min_float(a: f32, b: f32) -> f32 {
    if a <= b {
        a
    } else {
        b
    }
}

pub fn max_float(a: f32, b: f32) -> f32 {
    if a >= b {
        a
    } else {
        b
    }
}

/// Returns all rows containing the given collider.
fn get_min_max_of_collider(
    collider: &Collider,
    global_transform: &GlobalTransform,
) -> Option<(f32, f32)> {
    collider
        .as_convex_polygon()
        .and_then(|view| get_min_max(view.points().map(|v| v.y)))
}

pub(crate) const ROWS: u8 = 18;
pub(crate) const COLUMNS: u8 = 10;

#[derive(Event, Debug)]
struct RowIntersections(HitMap);

fn clear_hit_map(
    //mut unfreeze: EventReader<UnFreeze>,
    mut hitmap: ResMut<HitMap>,
) {
    //if !unfreeze.is_empty() {
    //   unfreeze.clear();
    hitmap.0.clear();
    //}
}

fn clear_partition_map(
    //mut unfreeze: EventReader<UnFreeze>,
    mut partition: ResMut<Partitions>,
) {
    //if !unfreeze.is_empty() {
    //   unfreeze.clear();
    partition.0.clear();
    //}
}

#[derive(Debug, Clone, Copy)]
pub struct RowBounds {
    pub lower: f32,
    pub upper: f32,
    pub row: u8,
}

impl RowBounds {
    /// Returns `(upper: f32, lower: f32)` bounds in global game space for a given
    /// `Row`. This centralizes row bound calculation to avoid the odd arithmetic error.
    ///
    /// `Row` are zero-indexed.
    pub(crate) fn new(row: Row) -> Option<Self> {
        if row <= ROWS {
            let lower = GROUND_Y + BRICK_DIM * f32::from(row);
            let upper = lower + BRICK_DIM;
            Some(RowBounds { upper, lower, row })
        } else {
            None
        }
    }

    /// Creates `RowBound` from y-coordinate using `GROUND_Y`.
    ///
    /// NB. If `y` has penetrated the ground the conversion to `u8` acts as a
    /// floor function.
    pub(crate) fn from_y(y: f32) -> Option<Self> {
        let row = (y - GROUND_Y) / BRICK_DIM;
        Self::new(row as u8)
    }

    pub(crate) fn rows_between(
        &self,
        other: &Self,
    ) -> impl Iterator<Item = u8> {
        let (lower, upper) = match self.row.cmp(&other.row) {
            Ordering::Equal => (self.row, self.row),
            Ordering::Less => (self.row, other.row),
            Ordering::Greater => (other.row, self.row),
        };
        lower..=upper
    }

    /// If row is 0, then it adds extra padding
    #[inline]
    pub fn contains_y(&self, y: f32) -> bool {
        let mut lower = self.lower;
        if self.row == 0 {
            lower -= 30.0;
        }
        lower <= y && y <= self.upper
    }

    pub fn contains_vec2(&self, v: Vec2) -> bool {
        self.contains_y(v.y)
    }

    pub fn contains_vec3(&self, v: Vec3) -> bool {
        self.contains_y(v.y)
    }
}

fn draw_rows(mut cmds: Commands) {
    for row in 0..ROWS {
        let left_x = BRICK_DIM * -10.0;
        let y = GROUND_Y + BRICK_DIM * f32::from(row);
        let origin = Vec2::new(left_x, y);
        let dest = Vec2::new(left_x + 5.0 * BRICK_DIM, y);
        draw_ray(cmds.reborrow(), origin, dest);
    }
}

/// Must have more than two points
type ConvexHull = Vec<Vec2>;

// 0 =< Row <= 17
type Row = u8;

/// Determines whether point's y-coordinate falls within a given `Row`.
///
/// Expects y-coordinate in global space.
///
/// NOTE: should check for x-bound and lower y
fn is_point_in_row(row: Row) -> impl FnMut(&Vec2) -> bool {
    move |&Vec2 { y, .. }| is_y_in_row(row)(&y)
}

/// Determines whether point's y-coordinate falls within a given `Row`.
///
/// Expects y-coordinate in global space.
///
/// NOTE: should check for x-bound and lower y
pub fn is_y_in_row(row: Row) -> impl FnMut(&f32) -> bool {
    let r: f32 = row.into();
    let upper = (r + 1.0) * BRICK_DIM + GROUND_Y;
    let lower = if row == 0 {
        GROUND_Y - BRICK_DIM * 5.0
    } else {
        r * BRICK_DIM + GROUND_Y
    };
    move |&y| -> bool { lower <= y && y <= upper }
}

/// Partition map of collider in row to its partitioned collider.
///
/// - Since each collider has at most one partition per row,
///   this allows us to drop the outer `Vec`
/// - Then to perform a row slice, just grab each tetroid body in the given
///   row, despawn the tetroid body, spawn two bodies for row+1 and row-1
///   with their respective group of convex hulls.
///
#[derive(Resource, Debug, Default)]
struct Partitions(HashMap<Entity, Vec<(Row, ConvexHull)>>);

fn partitions(
    mut partitions: ResMut<Partitions>,
    hitmap: ResMut<HitMap>,
    mut cmds: Commands,
    //rc: Res<RapierContext>,
    //tetroids: Query<Entity, (With<Tetroid>, With<RigidBody>)>,
    //children: Query<&Children, With<Tetromino>>,
    colliders: Query<
        (Entity, &Collider, &GlobalTransform),
        With<TetrominoCollider>,
    >,
    //global_transforms: Query<&GlobalTransform>,
    //mut slices: EventReader<SliceRows>,
    mut densities: EventWriter<RowDensity>,
    //parents: Query<&Parent>,
    //mut freeze: EventReader<Freeze>,
    //mut despawn: EventWriter<DespawnTetromino>,
) {
    //if freeze.is_empty() {
    //    return;
    //}
    //freeze.clear();

    // CHeck for row slices to apply
    // NOTE: may need to deduplicate
    //let rows_to_slice: Vec<u8> = slices
    //    .read()
    //    //.filter_map(|rd| {
    //    //    if rd.density >= 0.9 {
    //    //        Some(rd.row)
    //    //    } else {
    //    //        None
    //    //    }
    //    //})
    //    .map(|s| s.row)
    //    .sorted()
    //    .dedup()
    //    .collect();
    //slices.clear();
    //

    partitions.0.clear();

    // populate partitions from `HitMap`
    for row in 0..ROWS {
        let mut in_row = colliders_in_row(row, colliders.iter());
        in_row.sort_by_key(|(id, _)| *id);
        in_row.dedup();
        //if !in_row.is_empty() && !rows_to_slice.is_empty() && row == 0 {
        //    dbg!(&row, &in_row);
        //}

        let mut area = 0.0;
        let mut area_dbg = Vec::new();
        // FIXME: duplicating `points` insertion. idk how
        let mut last: Vec<Vec2> = Vec::new();
        in_row.clone().into_iter().for_each(|(id, mut points)| {
            // initialize with collider points within row after appling global
            // transform
            // FIXME: duplicate traversal of collider points for colliders that
            // intersect multiple rows
            // - How to add points to rows above and below without duplicating
            // the insertions in other passes?
            // add lower ray hits

            // remove points not in row
            points.retain(is_point_in_row(row));

            if let Some(hits) = hitmap.0.get(&(id, row)) {
                points.extend_from_slice(hits);
            }
            // add upper ray hits
            if let Some(hits) = hitmap.0.get(&(id, row + 1)) {
                points.extend_from_slice(hits);
            }

            if last == points {
                //warn!("found dup: points == last: {:?}", &points);
            }
            last.clone_from(&points);

            // Add ColliderView points and hitmap hits to partition entry for
            // key (collider, row)
            sort_convex_hull(&mut points);
            if let Some(hull_area) = convex_hull_area(&points) {
                if std::panic::catch_unwind(||
                    Collider::convex_hull(&points)).is_err() {
                    error!("parry2d panic: failed to convexrt point cloud to convex hull") ;
                    dbg!(&points);
                } else if Collider::convex_hull(&points).is_some() {
                    area += hull_area;
                    // NOTE: The area is wrong because when it degrades (idk
                    // why it doesn't fuck up initially), it duplicates hulls
                    // and double counts them
                    area_dbg.push(points.clone());
                }
            }

            // Update `Partition`
            if let Some(row_entries) = partitions.0.get_mut(&id) {
                // NOTE: overwrite/ignore possibly existing entry for current
                // entity as there can (should, rather) be only one of a given
                // collider per row
                row_entries.push((row, points.clone()))
            } else {
                let v = vec![(row, points.clone())];
                partitions.0.insert(id, v);
            }
        });

        // Calculate density
        let total_area = f32::from(COLUMNS) * BRICK_DIM * BRICK_DIM;
        //dbg!(area, total_area);
        let density = area / total_area;
        if density > 1.0 {
            //  dbg!(&area_dbg);
        }
        //let density_dbg = area_dbg
        //    .iter()
        //    .filter_map(convex_hull_area)
        //    .dedup()
        //    .sum::<f32>()
        //    / total_area;
        let rd = RowDensity { row, density };
        densities.send(rd);
    }

    // NOTE: so bloody inelegant but this is a prototype after all:
    // - with a single loop, the partition table didn't get built up completely
    //   before trying to execute the `SliceRow`s
    // - TODO: Consider collecting a `parents_in_row: Vec<(Entity, Vec<Row>)`
    //   and then appling the slice per parent, allowing for proper application
    //   multiple `SliceRow`s to a single `Tetromino`/parent. As a bonus it
    //   would be faster.

    //if freeze.is_empty() {
    //    return;
    //}
    //freeze.clear();
    //for row in 0..ROWS {
    //    // Slice logic: Triggers if there is a `SliceRow` AND a `Freeze`, as we
    //    // only want to slice on ground contact.
    //    if rows_to_slice.contains(&row) {
    //        freeze.clear();
    //
    //        let in_row = colliders_in_row(row, colliders.iter());
    //        let parents_in_row: Vec<Entity> = in_row
    //            .iter()
    //            .flat_map(|(child, _)| parents.iter_ancestors(*child))
    //            .sorted()
    //            .dedup()
    //            .collect();
    //
    //        let hull_to_collider =
    //            |hull: &ConvexHull| Collider::convex_hull(hull).unwrap();
    //
    //        // for each parent, get children, lookup (child_id, row) in
    //        // partitions and collect ConvexHulls
    //        // - colliders are new and should be spawned under a rigidbody
    //        for parent in parents_in_row {
    //            let mut hulls = vec![];
    //
    //            for child in children.iter_descendants(parent) {
    //                // - if child collider is in row, fetch slices from
    //                //   `.Partition`,` and add to`.`.hulls```
    //                // - otherwise,`.leave `apply global transform and add to
    //                //   `hulls`
    //
    //                if let Some(entries) = partitions.0.get(&child) {
    //                    // if child is in row, use partitions, otherwise, fetch
    //                    // existing colliders
    //                    let is_child_in_row =
    //                        in_row.iter().map(|(id, _)| id).contains(&child);
    //
    //                    let rows = entries
    //                        .iter()
    //                        .map(|(r, _)| r)
    //                        .collect::<Vec<&u8>>();
    //
    //                    entries.iter().for_each(|(collider_row, hull)| {
    //                        if is_child_in_row {
    //                            match Ord::cmp(&row, collider_row) {
    //                                Ordering::Equal => {}
    //                                _ => {
    //                                    hulls.push(hull.clone());
    //                                }
    //                            }
    //                        } else if let Ok((_, col, transform)) =
    //                            colliders.get(child)
    //                        {
    //                            let view = col.as_convex_polygon().unwrap();
    //                            let global_points: Vec<Vec2> = view
    //                                .points()
    //                                .map(|p| transform_point(transform)(&p))
    //                                .collect();
    //
    //                            match Ord::cmp(&row, collider_row) {
    //                                Ordering::Equal => {}
    //                                _ => {
    //                                    hulls.push(global_points);
    //                                }
    //                            }
    //                        }
    //                    });
    //                }
    //            }
    //
    //            // With `spawn_hull_groups` working, can we not just collect
    //            // all the new hulls into a single `Vec` and pass it to
    //            // `spawn_hull_groups`
    //            //
    //            // This shit works.
    //            spawn_hull_groups(cmds.reborrow(), hulls);
    //
    //            // FIXME: somehow this is running multiple times and causing
    //            // panics. I'm pretty sure it panics when multiple rows that each
    //            // cantain the same parent are sliced. The first slice despawns
    //            // the parent, the second somehow manages to get through the
    //            // above logic (which needs the parent i'm pretty sure), and
    //            // then goes and tries to despawn it again and, voilà, we have
    //            // a panic.
    //            // - Handle despawn in separate system scheduled after
    //            // `partitions`, and deduplicate `DespawnTetromino` events?
    //            //
    //            //
    //            // despawn parent
    //            if let Some(mut parent_cmds) = cmds.get_entity(parent) {
    //                //info!("Despawing Tetromino: parent id = {:?}", &parent);
    //                //parent_cmds.despawn_descendants();
    //                //parent_cmds.despawn()
    //                info!("Event: DespawnTetromino({:?})", &parent);
    //                despawn.send(DespawnTetromino(parent));
    //            } else {
    //                warn!("No Tetromino parent to despawn: id = {:?}", parent);
    //            };
    //
    //            //}
    //        }
    //
    //        // done slicing, so wipe row densities
    //        cmds.trigger(ClearRowDensities);
    //
    //        // TODO:
    //        // [ ] sort out event order for spawning blocks (relying on
    //        //     hacky freeze trigger)--it seems to hang after only triggered
    //        //     `SliceRow`s
    //        // [ ] replace spawn_lblock with spawn_random_block
    //        // [ ] try to load in mesh
    //        //
    //        //spawn_lblock(cmds.reborrow());
    //    }
}

fn despawn_tetrominos(
    mut despawn: EventReader<DespawnTetromino>,
    mut cmds: Commands,
) {
    despawn
        .read()
        .sorted()
        .dedup()
        .for_each(|&DespawnTetromino(id)| {
            // NOTE: for some reason this bugged the fuck out when used with
            // the `if let ... cmds.get_entity`. The only other change made was
            // adding a `!freeze.is_empty()` guard to `check_row_densities`
            // before sending `SliceRow` events. Probably that right?

            //if let Some(mut parent_cmds) = cmds.get_entity(id) {
            //info!("Despawning Tetromino: id = {:?}", &id);
            cmds.entity(id).despawn_recursive();
            //} else {
            //    warn!(
            //        "Attempted to despawn non-existent Tetromino: id = {:?}",
            //        id
            //    );
            //};
        });
}

/// Check for disconnected shapes.
///
/// For each group of connected hulls, spawn a `RigidBody` whose children are
/// the members of the group.
fn spawn_hull_groups(
    mut cmds: Commands,
    convex_hulls: Vec<(ColliderData, Vec<Vec2>)>,
    mut images: &mut Assets<Image>,
    mut slice_images: &mut EventWriter<SliceImage>,
    slice_rows: &SliceRows,
) {
    //warn!("Spawning hull groups");
    // Note: bounds are in global space to avoid conversion back in
    // `apply_slice_image`
    let groups = group_hulls(convex_hulls).into_values();
    for group in groups {
        assert!(!group.is_empty());
        let colliders = group.into_iter().filter_map(|(collider_data, h)| {
            if h.len() > 2 {
                let mut x_bounds: Option<(f32, f32)> = None;
                let mut y_bounds: Option<(f32, f32)> = None;
                let ps: Vec<Vec2> = h
                    .into_iter()
                    .map(|p| {
                        // set bounds
                        x_bounds = x_bounds.map_or(
                            Some((p.x, p.x)),
                            |(left, right)| {
                                Some((
                                    min_float(p.x, left),
                                    max_float(p.x, right),
                                ))
                            },
                        );
                        y_bounds = y_bounds.map_or(
                            Some((p.y, p.y)),
                            |(left, right)| {
                                Some((
                                    min_float(p.y, left),
                                    max_float(p.y, right),
                                ))
                            },
                        );
                        // invert global transform to get vertices in local
                        // space
                        let inverse = collider_data
                            .global_transform
                            .compute_matrix()
                            .inverse();
                        let ip = inverse.transform_point(p.extend(0.0));
                        ip.xy()
                    })
                    .collect();
                match Collider::convex_hull(&ps) {
                    Some(collider) => {
                        Some((collider_data, collider, x_bounds, y_bounds))
                    }
                    None => {
                        warn!("Found non-convex hull: {:?}", &ps);
                        None
                    }
                }
            } else {
                warn!(
                    "Found non-convex hull with two or fewer points: {:?}",
                    &h
                );
                None
            }
        });

        // TODO:
        //slice_images.send(SliceImage {
        //    image_handle,
        //    rows_to_slice: rows_to_slice
        //        .clone()
        //        .into_iter()
        //        .cloned()
        //        .collect(),
        //});

        // Post slice chunks get full gravity.
        cmds.spawn(TetrominoBundle::new(1.0))
            .with_children(|children| {
                colliders.for_each(
                    |(collider_data, col, x_bounds, y_bounds)| {
                        // FIXME:
                        // - we need the old collider's transform, since the hull
                        //   vertices we use are actually in global space, the
                        //   sprite inherits its transform correctly, but then
                        //   renders at the origin.
                        //   - The fix: apply old collider transform to new points,
                        //     create collider with old transform.
                        // - we also need to figure out how to send `SliceImage`.
                        //   Could we move it back to `apply_slices`?

                        //dbg!(&x_bounds);
                        //dbg!(&y_bounds);
                        let Some(x_bounds) = x_bounds else {
                            return;
                        };
                        let Some(y_bounds) = y_bounds else {
                            return;
                        };

                        // Apply transform to collider points.
                        let id = children
                            .spawn(
                                TetrominoColliderBundle {
                                    collider: col,
                                    ..default()
                                }
                                .with_friction(FRICTION)
                                .with_transform(
                                    collider_data
                                        .global_transform
                                        .compute_transform(),
                                ),
                            )
                            .with_children(|collider_children| {
                                //warn!("spawning sprite bundle");
                                let mut sprite_bundle =
                                    image_handle_to_sprite_bundle(
                                        collider_data.image_handle,
                                    );
                                // NOTE: appling global_transform worked for the
                                // first slice applied to a collider. But on the
                                // second, the image would spawn at the origin.
                                //
                                //sprite_bundle.1.transform =
                                //collider_data.global_transform.compute_transform();
                                let sprite_id = collider_children
                                    .spawn(sprite_bundle)
                                    .id();

                                slice_images.send(SliceImage {
                                    sprite_id,
                                    slice_rows: slice_rows.clone(),
                                    x_bounds,
                                    y_bounds,
                                });
                            })
                            .id();
                        // send SliceImage event where
                        // - SliceImage { former_handle_owner: Entity, }
                        // - ah fuck we need the rows that were sliced to clear the
                        //   pixels
                    },
                )
            });
    }
}

// Check if two `ConvexHull`s are contiguous by checking for any shared vertex.
// This only works because of the convex slices and limited convexity of the
// tetroids.
//
// Usess `max_error = 0.0` for fuzzy equality check.
fn any_shared_point(ls: &[Vec2], rs: &[Vec2]) -> bool
where
{
    let max_error = 0.1;
    ls.iter().any(|l| {
        rs.iter().any(|r| {
            ((r.y - l.y).abs() <= max_error)
                && ((r.x - l.x).abs() <= max_error)
        })
    })
}

fn group_hulls(
    hulls: Vec<(ColliderData, ConvexHull)>,
) -> HashMap<usize, Vec<(ColliderData, ConvexHull)>> {
    let mut groups: HashMap<usize, Vec<(ColliderData, ConvexHull)>> =
        HashMap::new();
    let mut group_counter: usize = 0;

    for (collider_data, hull) in hulls {
        // check if hull is connected to existing group
        let mut in_group = None;
        for (id, group) in groups.iter_mut() {
            for (_, h) in group.iter() {
                if any_shared_point(&hull, h) {
                    in_group = Some(*id);
                }
            }
        }
        match in_group {
            // if `hull` not found in existing group,
            // -> create new group containing `hull`
            None => {
                groups.insert(group_counter, vec![(collider_data, hull)]);
                group_counter += 1;
            }
            // otherwise add `hull` to group
            Some(id) => {
                if let Some(v) = groups.get_mut(&id) {
                    v.push((collider_data, hull));
                }
            }
        }
        //in_group = None;
    }

    groups
}

fn send_slice(
    //mut slices: EventWriter<SliceRow>,
    mut commands: Commands,
    mut unfreeze: EventWriter<UnFreeze>,
) {
    // FIXME: Yea it's fucking broken. Not even worth debugging the area bug
    // until we sort out multiple slices
    commands.trigger(SliceRows { rows: vec![0, 1] });
    // slices.send(SliceRow { row: 2 });
    unfreeze.send(UnFreeze);
}

#[derive(Event, Debug, Eq, PartialEq, PartialOrd, Ord)]
pub struct SliceRow {
    pub row: u8,
}

#[derive(Event, Clone, Debug, Eq, PartialEq, PartialOrd, Ord)]
pub struct SliceRows {
    pub rows: Vec<u8>,
}

impl SliceRows {
    pub fn new(rows: Vec<u8>) -> Option<Self> {
        if !rows.is_empty() {
            Some(SliceRows { rows })
        } else {
            None
        }
    }

    /// Return rows in `self.rows` within the given row bounds.
    ///
    /// Used to select which slices to apply a tetroid with the given bounds.
    pub fn rows_within<'a>(
        &'a self,
        lower: &'a RowBounds,
        upper: &'a RowBounds,
    ) -> impl Iterator<Item = &u8> + 'a {
        if lower.row > upper.row {
            dbg!(&lower, &upper);
        }
        // FIXME: idx how but this panicked once
        assert!(lower.row <= upper.row);
        self.rows
            .iter()
            .filter(move |&&r| r >= lower.row && r <= upper.row)
    }
}

/// Indicates that a parent `Tetromino` should be despawned along with all of
/// its children.
#[derive(Event, Debug, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct DespawnTetromino(Entity);

/// Get area of clockwise-ordered convex hull.
///
/// See
/// [algorithm](https://stackoverflow.com/questions/451426/how-do-i-calculate-the-area-of-a-2d-polygon)
///
/// NOTE: untested
fn convex_hull_area(convex_hull: &ConvexHull) -> Option<f32> {
    if convex_hull.len() > 2 {
        Some(
            0.5 * zip(
                convex_hull,
                convex_hull[1..].iter().chain(once(&convex_hull[0])),
            )
            .fold(0.0, |area, (p0, p1)| area + p0.x * p1.y - p1.x * p0.y)
            .abs(),
        )
    } else {
        None
    }
}

/// Update table of intersection points of each `TetroidCollider` (so convex
/// child colliders of `Tetromino`s) with every row.
///
///
//
//             v0       v1
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
fn row_intersections(
    mut hitmap: ResMut<HitMap>,
    rc: Res<RapierContext>,
    tetroids: Query<Entity, (With<TetrominoCollider>, With<Collider>)>,
    colliders: Query<&Collider, With<TetrominoCollider>>,
    global_transforms: Query<&GlobalTransform, With<Collider>>,
) {
    {
        hitmap.0.clear();
        //if freeze.is_empty() {
        //    return;
        //}
        //freeze.clear();

        // map of row to colliding entities
        //let row_map: HashMap<Row, Vec<Entity>> = HashMap::new();
        // FIXME: clear on event to avoid wiping resource within two frames
        let left_x = BRICK_DIM * -5.0;
        let right_x = BRICK_DIM * 5.0;
        // NOTE: excludes very top row
        for ray in 1..ROWS {
            let left_origin =
                Vec2::new(left_x, GROUND_Y + BRICK_DIM * f32::from(ray));
            let right_origin =
                Vec2::new(right_x, GROUND_Y + BRICK_DIM * f32::from(ray));
            let left_dir = Vec2::X;
            let right_dir = Vec2::X * -1.0;
            let max_toi = 300.0;
            let solid = true;
            let filter = QueryFilter::only_dynamic();
            // TODO: duplicate handling?
            // FIXME: the bug hunt for dups starts here
            let mut on_hit = |entity: Entity, ri: RayIntersection| {
                match hitmap.0.get_mut(&(entity, ray)) {
                    Some(hits) => hits.push(ri.point),
                    None => {
                        hitmap.0.insert((entity, ray), vec![ri.point]);
                    }
                }
                true
            };

            //draw_ray(cmds.reborrow(), left_origin, right_origin)
            rc.intersections_with_ray(
                left_origin,
                left_dir,
                max_toi,
                solid,
                filter,
                &mut on_hit,
            );
            rc.intersections_with_ray(
                right_origin,
                right_dir,
                max_toi,
                solid,
                filter,
                &mut on_hit,
            );
        }
        // populate partitions from `HitMap`
        //partitions.0.clear();
        //hitmap.0.clear();
    }
}

fn show_duplicates_in_hitmap(hitmap: Res<HitMap>)
//-> HashMap<String, ((f32, f32), usize)>
{
    let mut dups: HashMap<String, ((f32, f32), usize)> = HashMap::new();
    hitmap
        .0
        .values()
        .flatten()
        .map(|&Vec2 { x, y }| (x, y))
        .for_each(|tup @ (x, y)| {
            dups.entry(format!("{:?}", &tup))
                .and_modify(|t| t.1 += 1)
                .or_insert((tup, 1));
        });

    // NOTE:  so many found twice, once it degrades 4 is common, 8 creeps in
    // I think it fine. It's just what happens when two colliders touch. Maybe
    for (k, (t, count)) in dups.iter() {
        if *count > 2 {
            info!("Found pair ({},{}) {} times", t.0, t.1, count);
        }
    }

    //dups
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
    next_tetroid.send(SpawnNextTetromino);
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
    next_tetroid.send(SpawnNextTetromino);
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
    next_tetroid.send(SpawnNextTetromino);
}

/// If no active tetroid, send spawn event.
fn check_for_active_tetroid(
    active_tetroid: Query<Entity, With<ActiveTetromino>>,
    mut next_tetroid: EventWriter<SpawnNextTetromino>,
) {
    if active_tetroid.iter().next().is_none() {
        next_tetroid.send(SpawnNextTetromino);
        info!("SpawnNextTetroid")
    }
}

/// If no active tetroid, send spawn event.
fn list_active_tetroid(active_tetroid: Query<Entity, With<ActiveTetromino>>) {
    let len = active_tetroid.iter().len();
    if len > 1 {
        warn!("Too many active tetroids: expected 1 but found {}", len);
    }
}
