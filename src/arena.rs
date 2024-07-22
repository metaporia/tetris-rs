// arena.rs

use bevy::prelude::*;
use bevy::{color::prelude::*, sprite::Anchor};
use bevy_prototype_lyon::{
    draw::{Fill, Stroke},
    entity::ShapeBundle,
    geometry::GeometryBuilder,
    shapes::{self},
};
use bevy_rapier2d::prelude::*;

use crate::event_demo::{RowBounds, GROUND_Y, ROWS};
use crate::tetroid::BRICK_DIM;

const OUTLINE_THICKNESS: f32 = 1.0;

#[derive(Component)]
pub struct Ground;

#[derive(Component, Debug)]
pub enum Wall {
    Left,
    Right,
}

/// A `Vec<Enitity>` wrapper. The nth `Entity` represents the nth row's
/// `DensityIndicatorSquare`. Should be initialized such that
/// `RowDensityIndicatorMap.0 <= ROWS`.
#[derive(Resource, Debug, Default)]
pub struct RowDensityIndicatorMap(Vec<Entity>);

impl RowDensityIndicatorMap {
    fn get_by_row(&self, row: u8) -> Option<&Entity> {
        self.0.get(usize::from(row))
    }
}

#[derive(Component)]
pub(crate) struct DensityIndicatorSquare;

#[derive(Component)]
struct DensityIndicatorColumn;

#[derive(Event, Debug)]
pub(crate) struct RowDensity {
    pub row: u8,
    pub density: f32,
}

#[derive(Component)]
pub(crate) struct RowDensityText;

/// Update row density indicator column based on latest row densities.
/// TODO:
/// - add `RowDensity` events
/// - add `density_map_manager` to read `RowDensity` events and update the map
///   from row -> density
/// - fire `RowDensity` events at the end of the `partition` pipeline
/// NOTE: Add `Row` to indicator map? No the index works, right?
/// - And then we read `RowDensity(Row, Density)` events and apply each,
/// indexing `density_map` accordingly.
pub fn render_row_density(
    density_map: ResMut<RowDensityIndicatorMap>,
    mut squares: Query<(&mut Fill, &Children), With<DensityIndicatorSquare>>,
    mut densities: EventReader<RowDensity>,
    mut texts: Query<&mut Text, With<RowDensityText>>,
) {
    //assert!(density_map.0.len() == 18);
    //dbg!(density_map.0.len());
    for r @ RowDensity { row, density } in densities.read() {
        if let Some(RowBounds { lower, upper, row }) = RowBounds::new(*row) {
            if let Some(id) = density_map.get_by_row(row) {
                if let Ok((mut fill, children)) = squares.get_mut(*id) {
                    // update color
                    if let Color::Srgba(mut srgba) = fill.color {
                        srgba.alpha = *density;
                        fill.color = Color::Srgba(srgba)
                    }
                    // update text. `children` should be a singleton
                    if let Some(mut text) = children
                        .iter()
                        .next()
                        .and_then(|&id| texts.get_mut(id).ok())
                    {
                        let density_string = format!("{:.2}", density);

                        match &mut text.sections[..] {
                            [] => text.sections.push(TextSection::new(
                                density_string,
                                TextStyle::default(),
                            )),
                            [section] => section.value = density_string,
                            _ => {}
                        }
                    }
                }
            }
        }
    }
}

/// Density indicator column
/// - made up of a square for each row
pub fn spawn_density_indicator_column(
    mut commands: Commands,
    mut density_map: ResMut<RowDensityIndicatorMap>,
) {
    let extents = Vec2::new(BRICK_DIM, BRICK_DIM);
    let x = BRICK_DIM * -6.5;
    let square = shapes::Rectangle {
        extents,
        ..Default::default()
    };

    let transform_bundle = TransformBundle::from(Transform::from_xyz(
        x,
        GROUND_Y + BRICK_DIM * 0.5,
        0.0,
    ));

    commands
        .spawn(DensityIndicatorColumn)
        .insert(InheritedVisibility::VISIBLE)
        .insert(transform_bundle)
        .with_children(|column| {
            for row in 0..ROWS {
                if let Some(RowBounds { lower, upper, .. }) =
                    RowBounds::new(row)
                {
                    //let id = column.spawn() ;
                    let shape = (
                        ShapeBundle {
                            path: GeometryBuilder::build_as(&square),
                            ..default()
                        },
                        Fill::color(Color::Srgba(Srgba::new(
                            0.0, 0.0, 0.0, 0.0,
                        ))),
                        Stroke::new(
                            Color::Srgba(Srgba::BLACK),
                            OUTLINE_THICKNESS,
                        ),
                    );
                    let transform_bundle =
                        TransformBundle::from(Transform::from_xyz(
                            0.0,
                            f32::from(row) * BRICK_DIM,
                            0.0,
                        ));
                    let id = column
                        .spawn(shape)
                        .insert(transform_bundle)
                        .insert(DensityIndicatorSquare)
                        // add text bundle to render density number
                        .with_children(|children| {
                            // spawn initial empty text
                            children
                                .spawn(Text2dBundle {
                                    transform: Transform::from_xyz(
                                        -BRICK_DIM, 0.0, 0.0,
                                    ),
                                    text_anchor: Anchor::CenterRight,
                                    ..Default::default()
                                })
                                .insert(RowDensityText);
                        })
                        .id();
                    density_map.0.push(id);
                }
            }
        });
}

pub fn spawn_arena(mut commands: Commands) {
    let brick_half: f32 = BRICK_DIM / 2.0;

    let ground_extents = Vect {
        x: BRICK_DIM * 5.0,
        y: BRICK_DIM / 2.0,
    };
    let ground_origin_y = -BRICK_DIM * (18.0 / 2.0) - 0.5 * BRICK_DIM;
    let ground_shape = shapes::Rectangle {
        extents: Vect {
            x: ground_extents.x * 2.0 - OUTLINE_THICKNESS,
            y: ground_extents.y * 2.0 - OUTLINE_THICKNESS,
        },
        ..Default::default()
    };
    /* Create the ground. */
    commands
        //.spawn((
        //    ShapeBundle {
        //        path: bevy_prototype_lyon::geometry::GeometryBuilder::build_as(
        //            &ground_shape,
        //        ),
        //        ..default()
        //    },
        //    Fill::color(Color::BLACK),
        //    Stroke::new(Color::BLACK, OUTLINE_THICKNESS),
        //))
        .spawn(RigidBody::Fixed)
        .insert(ColliderMassProperties::Mass(100.0))
        .insert(Collider::cuboid(ground_extents.x, ground_extents.y))
        .insert(Restitution::coefficient(0.0))
        .insert(Friction::new(0.0))
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
        //.spawn((
        //    ShapeBundle {
        //        path: GeometryBuilder::build_as(&wall_shape),
        //        ..default()
        //    },
        //    Fill::color(Color::BLACK),
        //    Stroke::new(Color::BLACK, OUTLINE_THICKNESS),
        //))
        .spawn(Wall::Left)
        .insert(Collider::cuboid(brick_half, BRICK_DIM * 9.5))
        .insert(Friction::new(0.0))
        .insert(Restitution::coefficient(0.0))
        .insert(TransformBundle::from(Transform::from_xyz(
            -BRICK_DIM * 5.0 - brick_half,
            -BRICK_DIM * 0.5,
            0.0,
        )));

    // right wall
    commands
        .spawn((
            ShapeBundle {
                path: GeometryBuilder::build_as(&wall_shape),
                ..default()
            },
            //Fill::color(Color::Srgba(Srgba::BLACK)),
            Stroke::new(Color::Srgba(Srgba::BLACK), OUTLINE_THICKNESS),
        ))
        .insert(Collider::cuboid(brick_half, BRICK_DIM * 9.5))
        .insert(Restitution::coefficient(0.0))
        .insert(Friction::new(0.0))
        .insert(Wall::Right)
        .insert(TransformBundle::from(Transform::from_xyz(
            BRICK_DIM * 5.0 + brick_half,
            -BRICK_DIM * 0.5,
            0.0,
        )));
}
