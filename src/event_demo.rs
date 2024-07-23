//! Event loop deme
//!
//! An attempt to sort out SpawnTetroid -> (HitDebris | HitGround) loop
//!
//! Tetroid spawns, falls and once it hits the ground or debris the `Freeze`
//! event is sent. Once all calculations are done, the tetroid is deactivated,
//! `Unfreeze` sends, the physics simulation unpauses, and `SpawnTetroid` is
//! sent.

use crate::{draw_ray, Pause};
use bevy::input::common_conditions::input_just_pressed;
use bevy::log::{Level, LogPlugin};
use bevy::utils::HashMap;
use bevy::{prelude::*, window::WindowResolution};
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_prototype_lyon::entity::Path;
use bevy_prototype_lyon::plugin::ShapePlugin;
use bevy_rapier2d::prelude::*;

use itertools::Itertools;
use std::cmp::Ordering;
use std::iter::{once, zip};

use crate::arena::{
    self, render_row_density, spawn_density_indicator_column, Ground,
    RowDensity, RowDensityIndicatorMap,
};
use crate::tetroid::{
    components::*, spawn_lblock, TetroidColliderBundle, TetrominoBundle,
    BRICK_DIM,
};
use crate::{draw_convex_hull, kbd_input, sort_convex_hull, v2_to_3, v3_to_2};

#[derive(Event, Debug)]
struct Freeze;

#[derive(Event, Debug)]
struct UnFreeze;

#[derive(Event, Debug)]
struct DeactivateTetroid;

#[derive(Event, Debug)]
struct SpawnNextTetroid;

#[derive(Component, Debug, Default)]
pub(crate) struct Tetroid;

const PIXELS_PER_METER: f32 = 50.0;
const GRAVITY: f32 = 0.05;
const VELOCITY: Velocity = Velocity {
    linvel: Vec2::new(0.0, -100.0),
    angvel: 0.0,
};
pub(crate) const FRICTION: f32 = 0.3;

pub(crate) const GROUND_Y: f32 = -BRICK_DIM * 18.0 / 2.0;

pub fn app() {
    let mut app = App::new();
    app
        .add_event::<Freeze>()
        .add_event::<UnFreeze>()
        .add_event::<DeactivateTetroid>()
        .add_event::<Pause>()
        .add_event::<RowDensity>()
        .add_event::<SliceRow>()
        .add_event::<SpawnNextTetroid>()
        .add_event::<DespawnTetromino>()
        .insert_resource(HitMap::default())
        .insert_resource(Partitions::default())
        .insert_resource(RowDensityIndicatorMap::default())
        .add_plugins(
            DefaultPlugins.build()
                .set(WindowPlugin {
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
                })
                //.disable::<LogPlugin>()
                .set(LogPlugin {
                    //filter: "tetris-rs=debug,bevy_ecs=debug".to_string(),
                    ..Default::default()
                }),
        )
        // Schedule graph
        //.add_plugins(bevy_mod_debugdump::CommandLineArgs)
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
        .add_systems(
            Startup,
            (
                spawn_lblock,
                arena::spawn_arena,
                //draw_rows,
                spawn_density_indicator_column,
            ),
        )
        .add_systems(
            Update,
            (
                kbd_input,
                toggle_pause,
                reset_game.run_if(input_just_pressed(KeyCode::KeyR)),
                reset_tetroids.run_if(input_just_pressed(KeyCode::KeyT)),
                reset_debug_shapes.run_if(input_just_pressed(KeyCode::KeyC)),
            ),
        )
        .add_systems(
            Update,
            (
                (
                    //tetroid_hit,
                    tetroid_hit_compound,
                    freeze,
                    (
                        row_intersections,
                        partitions,
                        despawn_tetrominos,
                        clear_hit_map,
                        clear_partition_map,
                        check_row_densities,
                    )
                        .chain(),
                    send_slice.run_if(input_just_pressed(KeyCode::Space)),
                    deactivate_tetroid,
                    unfreeze, // TODO consolidate unfreeze and handle_unfreeze
                    handle_unfreeze,
                    block_spawner,
                    //check_for_active_tetroid
                    list_active_tetroid,
                )
                    .chain(),
                render_row_density,
            ),
        )
        //        .add_systems(Startup, setup_physics)
        //.add_systems(Update, kbd_input)
        //.add_systems(Update, tetroid_spawner)
    ;

    //bevy_mod_debugdump::print_schedule_graph(&mut app, Update);

    app.run();
}

fn setup_graphics(mut commands: Commands) {
    // Add a camera so we can see the debug-render.
    commands.spawn(Camera2dBundle::default());
}

/// Send `Freeze` event if active tetroid hits ground or other tetroid.
fn tetroid_hit_compound(
    mut freezes: EventWriter<Freeze>,
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
            info!("Freeze");
            freezes.send(Freeze);
        }
    }
}

/// On ground contact remove all velocity & gravity for ActiveTetroid, remove
/// ActiveTetroid from entity
fn freeze(
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
            sleeping.sleeping = true;

            //info!("Freezing all tetroids, id: {:?}, {:?}", entity, sleeping);
            *vel = Velocity::zero();
            gravity.0 = 0.0;
        }
    }
}

fn handle_unfreeze(
    mut tetroids: Query<
        (Entity, &mut Velocity, &mut GravityScale, &mut Sleeping),
        With<Tetroid>,
    >,
    mut unfreeze: EventReader<UnFreeze>,
) {
    if !unfreeze.is_empty() {
        unfreeze.clear();
        //dbg!("in freeze, {}", tetroids.iter().len());
        for (entity, mut vel, mut gravity, sleeping) in tetroids.iter_mut() {
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
            deactivate.send(DeactivateTetroid);
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

fn block_spawner(
    mut cmds: Commands,
    mut freeze: EventReader<Freeze>,
    mut next_tetroid: EventReader<SpawnNextTetroid>,
) {
    if !freeze.is_empty() || !next_tetroid.is_empty() {
        freeze.clear();
        next_tetroid.clear();
        spawn_lblock(cmds.reborrow());
    }
}

type Ray = u8;

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

/// Return iterator of colliders in a given row. This is exhaustive and
/// includes colliders not hit by rays.
///
fn colliders_in_row<'a>(
    row: Row,
    colliders: impl Iterator<Item = (Entity, &'a Collider, &'a GlobalTransform)>,
) -> Vec<(Entity, Vec<Vec2>)> {
    //let r: f32 = row.into();
    //let upper = (r + 1.0) * BRICK_DIM + GROUND_Y;
    //let lower = if row == 0 {
    //    GROUND_Y - BRICK_DIM * 9.0
    //} else {
    //    r * BRICK_DIM + GROUND_Y
    //};
    //let in_row = |p: Vec2| -> bool { lower <= p.y && p.y <= upper };
    let mut cs: Vec<(Entity, Vec<Vec2>)> = Vec::new();
    colliders
        .into_iter()
        .filter_map(|(e, c, t)| c.as_convex_polygon().map(|view| (e, view, t)))
        .for_each(|(id, view, t)| {
            let ps: Vec<Vec2> = view
                .points()
                .map(|p| v3_to_2(&t.transform_point(v2_to_3(&p))))
                .collect();
            if ps.iter().any(is_point_in_row(row)) {
                let t = (id, ps);
                cs.push(t)
            }
        });

    cs
}

fn transform_point(
    global_transform: &GlobalTransform,
) -> impl FnMut(&Vec2) -> Vec2 + '_ {
    |p: &Vec2| v3_to_2(&global_transform.transform_point(v2_to_3(p)))
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

/// Returns all rows containing the given collider.
fn find_bounding_rows(
    collider: &Collider,
    global_transform: &GlobalTransform,
) -> Option<(f32, f32)> {
    if let Some((min, max)) = collider.as_convex_polygon().and_then(|view| {
        view.points().fold(None, |bounds, p| {
            let p = global_transform.transform_point(v2_to_3(&p));
            match bounds {
                None => Some((p.y, p.y)),
                Some((min, max)) => {
                    if p.y < min {
                        Some((p.y, max))
                    } else if p.y > max {
                        Some((min, p.y))
                    } else {
                        Some((min, max))
                    }
                }
            }
        })
    }) {
        //RowBounds::new(
    }
    None
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

#[derive(Debug)]
pub(crate) struct RowBounds {
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

    pub(crate) fn from_y(y: f32) -> Option<Self> {
        let row = (y - GROUND_Y) % BRICK_DIM;
        Self::new(row as u8)
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
    let r: f32 = row.into();
    let upper = (r + 1.0) * BRICK_DIM + GROUND_Y;
    let lower = if row == 0 {
        GROUND_Y - BRICK_DIM * 5.0
    } else {
        r * BRICK_DIM + GROUND_Y
    };
    move |&Vec2 { y, .. }| -> bool { lower <= y && y <= upper }
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
    rc: Res<RapierContext>,
    tetroids: Query<Entity, (With<Tetroid>, With<RigidBody>)>,
    children: Query<&Children>,
    colliders: Query<(Entity, &Collider, &GlobalTransform), With<Tetroid>>,
    global_transforms: Query<&GlobalTransform>,
    mut slices: EventReader<SliceRow>,
    mut densities: EventWriter<RowDensity>,
    parents: Query<&Parent>,
    mut freeze: EventReader<Freeze>,
    mut despawn: EventWriter<DespawnTetromino>,
) {
    //if freeze.is_empty() {
    //    return;
    //}
    //freeze.clear();

    // CHeck for row slices to apply
    // NOTE: may need to deduplicate
    let rows_to_slice: Vec<u8> = slices
        .read()
        //.filter_map(|rd| {
        //    if rd.density >= 0.9 {
        //        Some(rd.row)
        //    } else {
        //        None
        //    }
        //})
        .map(|s| s.row)
        .sorted()
        .dedup()
        .collect();
    slices.clear();

    // populate partitions from `HitMap`
    for row in 0..ROWS {
        let in_row = colliders_in_row(row, colliders.iter());
        if !in_row.is_empty() && !rows_to_slice.is_empty() && row == 0 {
            //dbg!(&row, &in_row);
        }

        // TODO: skip calculation when not slicing current row
        let parents_in_row = children_to_unique_parents(
            in_row
                .iter()
                .flat_map(|(child, _)| parents.iter_ancestors(*child)),
        );
        //dbg!(parents_in_row.len());
        //println!("in_row: len = {:?}", &in_row.len());
        let mut area = 0.0;
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

            // Add ColliderView points and hitmap hits to partition entry for
            // key (collider, row)
            sort_convex_hull(&mut points);
            area += convex_hull_area(&points);

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
        let density = area / total_area;
        let rd = RowDensity { row, density };
        if density > 0.0 {
            //dbg!(&rd);
        }
        densities.send(rd);
    }

    // NOTE: so bloody inelegant but this is a prototype after all:
    // - with a single loop, the partition table didn't get built up completely
    //   before trying to execute the `SliceRow`s
    // - Consider collecting a `parents_in_row: Vec<(Entity, Vec<Row>)` and
    //   then appling the slice per parent, allowing for proper application
    //   multiple `SliceRow`s to a single `Tetromino`/parent. As a bonus it 
    //   would be faster.

    //if freeze.is_empty() {
    //    return;
    //}
    //freeze.clear();
    for row in 0..ROWS {
        // Slice logic: Triggers if there is a `SliceRow` AND a `Freeze`, as we
        // only want to slice on ground contact.
        if rows_to_slice.contains(&row) {
            freeze.clear();

            let in_row = colliders_in_row(row, colliders.iter());
            let parents_in_row: Vec<Entity> = in_row
                .iter()
                .flat_map(|(child, _)| parents.iter_ancestors(*child))
                .sorted()
                .dedup()
                .collect();

            let hull_to_collider =
                |hull: &ConvexHull| Collider::convex_hull(hull).unwrap();

            for parent in parents_in_row {
                // for each partent, get children, lookup (child_id, row) in
                // partitions and collect ConvexHulls
                // - group children into above, within, below
                // - colliders are new and should be spawned under a rigidbody
                let mut above = vec![];
                let mut within = vec![];
                let mut below = vec![];

                //dbg!(children .iter_descendants(parent) .collect::<Vec<Entity>>());
                for child in children.iter_descendants(parent) {
                    // - if child collider is in row, fetch slices from `Partition`,
                    //   - add sub-/olliders to above, withiin, and below,
                    //     respectively
                    // - otherwise, leave intact and add to one of above,
                    //   within, and below
                    if let Some(entries) = partitions.0.get(&child) {
                        // if child is in row, use partitions, otherwise, fetch
                        // existing colliders
                        let is_child_in_row =
                            in_row.iter().map(|(id, _)| id).contains(&child);
                        if !rows_to_slice.is_empty() {
                            //info!( "row: {:?}, is_child_in_row: {:?}, id: {:?}", &row, &is_child_in_row, &child);
                        }

                        let rows = entries
                            .iter()
                            .map(|(r, _)| r)
                            .collect::<Vec<&u8>>();
                        entries.iter().for_each(|(collider_row, hull)| {
                            if is_child_in_row {
                                match Ord::cmp(&row, collider_row) {
                                    Ordering::Greater => {
                                        above.push(hull.clone());
                                    }
                                    Ordering::Equal => {
                                        within.push(hull.clone());
                                    }
                                    Ordering::Less => {
                                        below.push(hull.clone());
                                    }
                                }
                            } else if let Ok((_, col, transform)) =
                                colliders.get(child)
                            {
                                let view = col.as_convex_polygon().unwrap();
                                let global_points: Vec<Vec2> = view
                                    .points()
                                    .map(|p| transform_point(transform)(&p))
                                    .collect();

                                match Ord::cmp(&row, collider_row) {
                                    Ordering::Greater => {
                                        above.push(global_points);
                                    }
                                    Ordering::Equal => {
                                        within.push(global_points);
                                    }
                                    Ordering::Less => {
                                        below.push(global_points);
                                    }
                                }
                            }
                        });
                    }
                }

                // With `spawn_hull_groups` working, can we not just collect
                // all the new hulls into a single `Vec` and pass it to 
                // `spawn_hull_groups`
                if !above.is_empty() {
                    spawn_hull_groups(cmds.reborrow(), above);
                }

                // spawn below
                if !below.is_empty() {
                    spawn_hull_groups(cmds.reborrow(), below);
                }

                // FIXME: somehow this is running multiple times and causing
                // panics. I'm pretty sure it panics when multiple rows that each
                // cantain the same parent are sliced. The first slice despawns
                // the parent, the second somehow manages to get through the
                // above logic (which needs the parent i'm pretty sure), and
                // then goes and tries to despawn it again and, voilà, we have
                // a panic.
                // - Handle despawn in separate system scheduled after
                // `partitions`, and deduplicate `DespawnTetromino` events?
                //
                //
                // despawn parent
                if let Some(mut parent_cmds) = cmds.get_entity(parent) {
                    //info!("Despawing Tetromino: parent id = {:?}", &parent);
                    //parent_cmds.despawn_descendants();
                    //parent_cmds.despawn()
                    info!("Event: DespawnTetromino({:?})", &parent);
                    despawn.send(DespawnTetromino(parent));
                } else {
                    warn!("No Tetromino parent to despawn: id = {:?}", parent);
                };

                //}
            }
            // TODO:
            // [ ] sort out event order for spawning blocks (relying on
            //     hacky freeze trigger)--it seems to hang after only triggered
            //     `SliceRow`s
            // [ ] replace spawn_lblock with spawn_random_block
            // [ ] try to load in mesh
            //
            //spawn_lblock(cmds.reborrow());
        }
    }
    //partitions.0.clear();
    //hitmap.0.clear();
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
            info!("Despawning Tetromino: id = {:?}", &id);
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
/// For each group of connected hulls, spawn a `RigidBody` whose children are
/// the members of the group.
fn spawn_hull_groups(mut cmds: Commands, convex_hulls: Vec<Vec<Vec2>>) {
    let groups = group_hulls(convex_hulls).into_values();
    for group in groups {
        assert!(!group.is_empty());
        let colliders = group
            .into_iter()
            // FIXME: game threw panic here somehow. Where are the concave
            // hulls slipping through?
            .filter_map(|h| {
                if h.len() > 2 {
                    match Collider::convex_hull(&h) {
                        Some(collider) => Some(collider),
                        None => {
                            warn!("Found non-convex hull: {:?}", &h);
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
        // Post slice chunks get full gravity.
        cmds.spawn(TetrominoBundle::new(1.0))
            .with_children(|children| {
                colliders.for_each(|col| {
                    children.spawn(TetroidColliderBundle::new(col, FRICTION));
                })
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

fn group_hulls(hulls: Vec<ConvexHull>) -> HashMap<usize, Vec<ConvexHull>> {
    let mut groups: HashMap<usize, Vec<ConvexHull>> = HashMap::new();
    let mut group_counter: usize = 0;

    for hull in hulls {
        // check if hull is connected to existing group
        let mut in_group = None;
        for (id, group) in groups.iter_mut() {
            for h in group.iter() {
                if any_shared_point(&hull, h) {
                    in_group = Some(*id);
                }
            }
        }

        match in_group {
            // if `hull` not found in existing group,
            // -> create new group containing `hull`
            None => {
                groups.insert(group_counter, vec![hull]);
                group_counter += 1;
            }
            // otherwise add `hull` to group
            Some(id) => {
                if let Some(v) = groups.get_mut(&id) {
                    v.push(hull);
                }
            }
        }
        //in_group = None;
    }

    groups
}

fn send_slice(
    mut slices: EventWriter<SliceRow>,
    mut unfreeze: EventWriter<UnFreeze>,
) {
    slices.send(SliceRow { row: 0 });
    unfreeze.send(UnFreeze);
}

/// Check for rows above the density threshhold (0.9) and if so send `SliceRow`
/// event.
///
/// As far as system ordering, this runs after `partitions` so if there is a
/// `SliceRow`, partitions can apply it on the next `Update`.
fn check_row_densities(
    mut densities: EventReader<RowDensity>,
    mut deactivate: EventWriter<DeactivateTetroid>,
    mut slices: EventWriter<SliceRow>,
    mut freeze: EventReader<Freeze>,
) {
    densities.read().for_each(|rd| {
        if rd.density >= 0.7 && !freeze.is_empty() {
            freeze.clear();
            let slice_row = SliceRow { row: rd.row };
            info!("{:?}", &slice_row);
            slices.send(slice_row);
            //deactivate.send(DeactivateTetroid);
        }
    })
}

#[derive(Event, Debug, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct SliceRow {
    row: u8,
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
fn convex_hull_area(convex_hull: &ConvexHull) -> f32 {
    0.5 * zip(
        convex_hull,
        convex_hull[1..].iter().chain(once(&convex_hull[0])),
    )
    .fold(0.0, |area, (p0, p1)| area + p0.x * p1.y - p1.x * p0.y)
    .abs()
}

/// Update table of intersection points of each tetroid with every row.
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
    tetroids: Query<Entity, (With<Tetroid>, With<Collider>)>,
    colliders: Query<&Collider, With<Tetroid>>,
    global_transforms: Query<&GlobalTransform, With<Collider>>,
) {
    {
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
) {
    freeze.clear();
    // despawn
    tetroids
        .iter()
        .for_each(|t| cmds.entity(t).despawn_recursive());
    shapes
        .iter()
        .for_each(|s| cmds.entity(s).despawn_recursive());

    spawn_lblock(cmds);
}

fn reset_tetroids(
    mut cmds: Commands,
    tetroids: Query<Entity, With<Tetroid>>,
    mut freeze: EventReader<Freeze>,
) {
    freeze.clear();
    // despawn
    tetroids
        .iter()
        .for_each(|t| cmds.entity(t).despawn_recursive());
    spawn_lblock(cmds);
}

#[derive(Component)]
pub(crate) struct DebugShape;

fn reset_debug_shapes(
    mut cmds: Commands,
    debug_shapes: Query<Entity, With<DebugShape>>,
    mut freeze: EventReader<Freeze>,
) {
    freeze.clear();
    // despawn
    debug_shapes
        .iter()
        .for_each(|t| cmds.entity(t).despawn_recursive());
    spawn_lblock(cmds);
}

/// If no active tetroid, send spawn event.
fn check_for_active_tetroid(
    active_tetroid: Query<Entity, With<ActiveTetroid>>,
    mut next_tetroid: EventWriter<SpawnNextTetroid>,
) {
    if active_tetroid.iter().next().is_none() {
        next_tetroid.send(SpawnNextTetroid);
        info!("SpawnNextTetroid")
    }
}

/// If no active tetroid, send spawn event.
fn list_active_tetroid(active_tetroid: Query<Entity, With<ActiveTetroid>>) {
    let len = active_tetroid.iter().len();
    if len > 1 {
        warn!("Too many active tetroids: expected 1 but found {}", len);
    }
}
