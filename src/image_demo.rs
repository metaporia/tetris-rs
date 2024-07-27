//! # Image demo
//!
//! - Load in `Tetromino` square images
//! - attempt software pixel discard
//! - otherwise write simple fragment shader with custom material
//! - A third option would be a vertex shader (if it works in 2d) and adjust
//!   the clipping coordinates to exclude slices.

use bevy::sprite::Anchor;
use bevy::{prelude::*, render::render_asset::RenderAssetUsages};
use bevy_inspector_egui::inspector_egui_impls::register_std_impls;
use bevy_rapier2d::prelude::*;
use image::{DynamicImage, ImageBuffer, Pixel};

use crate::draw_circle_contact;
use crate::event_demo::GROUND_Y;
use crate::tetroid::*;
use crate::BRICK_DIM;

/// Let's first go the in-memory, software route and instead of loading actual
/// assets, lets just fill an image buffer with a color and see if we can trim
/// it as necessary for `SliceRows`.
/// - [ ] create image
/// - [ ] add to spritebundle
/// - [ ] attach to parent or child?
///
/// Note: the original nottetris2 written in lua keeps the one entire tetromino
/// image for each body in a table. When the colliders (shapes in love) are
/// sliced, it loops through all pixels checking if that point projection hits
/// a collider belonging to the given body, and, if so, discards it.
///
/// See [this reddit post](https://www.reddit.com/r/rust_gamedev/comments/17labcg/firsttime_bevy_user_trying_to_generate_an/)
/// for a starting point for dynamic image manipulation.
pub fn in_memory_image() {}

type ImageBuf = ImageBuffer<image::Rgba<u8>, Vec<u8>>;

/// Generate a `crate::tetroid::TetrominoType::SQUARE`-sized image filled with
/// blue.
pub fn blue_square() -> ImageBuf {
    let mut image = ImageBuffer::new(BRICK_DIM as u32, BRICK_DIM as u32);
    for (x, y, pixel) in image.enumerate_pixels_mut() {
        let p = image::Rgba::<u8>([0, 255, 255, 200]);
        *pixel = p;
    }
    image
}

pub fn register_and_return_sprite_bundle(
    image: ImageBuf,
    images: Res<AssetServer>,
) -> SpriteBundle {
    let image_handle: Handle<Image> = images.add(Image::from_dynamic(
        DynamicImage::from(image),
        false,
        RenderAssetUsages::RENDER_WORLD | RenderAssetUsages::MAIN_WORLD,
    ));

    SpriteBundle {
        texture: image_handle,
        //transform: Transform::from_xyz(BRICK_DIM / 2.0, BRICK_DIM / 2.0, 0.0),
        sprite: Sprite {
            anchor: Anchor::BottomLeft,
            ..default()
        },
        ..default()
    }
}

#[derive(Event, Debug)]
pub struct SpawnSquare;

// make simple square collider with image attached
pub fn spawn_blue_square(
    trigger: Trigger<SpawnSquare>,
    mut commands: Commands,
    images: Res<AssetServer>,
) {
    let collider = TetrominoType::square();
    let blue_square = blue_square();
    let collider_bundle = TetroidColliderBundle::new(collider, 0.0);
    let body_bundle =
        (TetrominoBundle::new(0.3), InheritedVisibility::VISIBLE);
    let sprite_bundle = register_and_return_sprite_bundle(blue_square, images);

    commands
        .spawn(body_bundle)
        .with_children(|children| {
            children.spawn(collider_bundle).log_components();
            children
                .spawn(sprite_bundle)
                .insert(SquareImage)
                .log_components();
        })
        .log_components();
}

#[derive(Component)]
pub struct SquareImage;

#[derive(Event, Debug)]
pub struct ClearBelow {
    pub y_cutoff: f32,
}

// TODO manually edit resource on slice
pub fn clear_below_y(
    trigger: Trigger<ClearBelow>,
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    mut square: Query<
        (Entity, &mut Handle<Image>, &Transform, &GlobalTransform),
        With<SquareImage>,
    >,
) {
    let ClearBelow { y_cutoff } = trigger.event();
    info!("{:?}", trigger.event());
    if let Ok((entity, mut image_handle, transform, global_transform)) =
        square.get_single_mut()
    {
        //let v = global_transform.transform_point(Vec3::new(0.0, (*y_cutoff as f32) + GROUND_Y, 0.0));
        // NOTE: rounding from cast could be an issue
        dbg!(y_cutoff);

        if let Some(mut image) = images.remove(image_handle.as_mut()) {
            //let width = image.texture_descriptor.size.width;
            //let height = image.texture_descriptor.size.height;
            let dims = image.size();
            let scaled_dims = dims.as_vec2() * transform.scale.truncate();
            let width = scaled_dims.x;
            let height = scaled_dims.y;

            // This works assuming that x & y are zero-indexed
            //
            // I'm pretty sure each field in rgba is a u8, so one pixel is a
            // `&[u8; 4]`. But the image origin is top left so coordinates need
            // to be flipped.
            for i in (0..(image.data.len() as u32)).step_by(4) {
                // NOTE: image origin is top left so positions need to be
                // flipped
                let y = height as u32 - i / (width as u32 * 4);
                let x = width as u32 - (i % (width as u32 * 4)) / 4;
                let local_p = Vec3::new(x as f32, y as f32, 0.0);
                let global_p = global_transform.transform_point(local_p);
                if global_p.y  < *y_cutoff {
                    image.data[i as usize + 3] = 0;
                    //draw_circle_contact( global_p.truncate(), commands.reborrow(),);
                }
            }

            *image_handle = images.add(image);
        }
    }
}

pub fn clear_pixels_below_y(
    y_cutoff: u32,
    image_buffer: &mut ImageBuffer<image::Rgba<u8>, Vec<u8>>,
) {
    for (x, y, pixel) in image_buffer.enumerate_pixels_mut() {
        // flip since origin is top left
        if y > y_cutoff {
            // set alpha channel to zero
            pixel[3] = 0;
        }
    }
}
