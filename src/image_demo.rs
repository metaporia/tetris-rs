//! # Image demo
//!
//! - Load in `Tetromino` square images
//! - attempt software pixel discard
//! - otherwise write simple fragment shader with custom material
//! - A third option would be a vertex shader (if it works in 2d) and adjust
//!   the clipping coordinates to exclude slices.

use std::slice::RChunksExactMut;

use bevy::render::render_resource::TextureFormat;
use bevy::sprite::Anchor;
use bevy::{prelude::*, render::render_asset::RenderAssetUsages};
use bevy_inspector_egui::inspector_egui_impls::register_std_impls;
use bevy_rapier2d::prelude::*;
use image::{DynamicImage, ImageBuffer, Pixel, RgbaImage};

use crate::draw_circle_contact;
use crate::event_demo::{RowBounds, GROUND_Y};
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
fn in_memory_image() {}

/// Generate a `crate::tetroid::TetrominoType::SQUARE`-sized image filled with
/// blue outlined in black.
pub fn blue_square() -> RgbaImage {
    let extent = BRICK_DIM as u32;
    let mut image = ImageBuffer::new(BRICK_DIM as u32, BRICK_DIM as u32);
    for (x, y, pixel) in image.enumerate_pixels_mut() {
        let mut p = image::Rgba::<u8>([0, 255, 255, 200]);
        // check if on perimeter
        // NOTE: unsure if this works yet as the debug covers the outline
        if x == 0 || y == 0 || x == extent || y == extent {
            // set to black
            p = image::Rgba::<u8>([0, 0, 0, 255]);
        }
        *pixel = p;
    }
    image
}

/// Add an image to the asset server and create a `SpriteBundle` with the
/// resultant `Handle<Image>`>
pub fn rgba_image_to_sprite_bundle(
    image: RgbaImage,
    mut images: &mut Assets<Image>,
) -> (SquareImage, SpriteBundle) {
    let image_handle: Handle<Image> = images.add(Image::from_dynamic(
        DynamicImage::from(image),
        false,
        RenderAssetUsages::RENDER_WORLD | RenderAssetUsages::MAIN_WORLD,
    ));

    (
        SquareImage,
        SpriteBundle {
            texture: image_handle,
            //transform: Transform::from_xyz(BRICK_DIM / 2.0, BRICK_DIM / 2.0, 0.0),
            sprite: Sprite {
                anchor: Anchor::BottomLeft,
                ..default()
            },
            ..default()
        },
    )
}

/// Add image to `Assets<Image>`, create new sprite bundle with the resultant
/// image handle.
pub fn image_to_sprite_bundle(
    image: Image,
    mut images: &mut Assets<Image>,
) -> (SquareImage, SpriteBundle) {
    let image_handle: Handle<Image> = images.add(image);
    image_handle_to_sprite_bundle(image_handle, images)
}

/// Add image to `Assets<Image>`, create new sprite bundle with the resultant
/// image handle.
pub fn image_handle_to_sprite_bundle(
    image_handle: Handle<Image>,
    mut images: &mut Assets<Image>,
) -> (SquareImage, SpriteBundle) {
    (
        SquareImage,
        SpriteBundle {
            texture: image_handle,
            sprite: Sprite {
                anchor: Anchor::BottomLeft,
                ..default()
            },
            ..default()
        },
    )
}

pub fn new_blue_square_bundle(
    mut images: &mut Assets<Image>,
) -> (SquareImage, SpriteBundle) {
    let blue_square = blue_square();
    rgba_image_to_sprite_bundle(blue_square, images)
}

#[derive(Event, Debug)]
pub struct SpawnSquare;

// make simple square collider with image attached
pub fn spawn_blue_square(
    trigger: Trigger<SpawnSquare>,
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
) {
    let collider = TetrominoType::square();
    let blue_square = blue_square();
    let collider_bundle = TetroidColliderBundle::new(collider, 0.0);
    let body_bundle =
        (TetrominoBundle::new(0.3), InheritedVisibility::VISIBLE);
    let sprite_bundle =
        rgba_image_to_sprite_bundle(blue_square, images.as_mut());

    commands
        .spawn(body_bundle)
        .with_children(|children| {
            children
                .spawn(collider_bundle)
                .insert(sprite_bundle)
                .insert(SquareImage)
                .log_components();
        })
        .log_components();
}

#[derive(Component, Default, Debug)]
pub struct SquareImage;

#[derive(Bundle, Default, Debug)]
pub struct SquareImageSpriteBundle {
    sprite_bundle: SpriteBundle,
    square_image: SquareImage,
}

impl SquareImageSpriteBundle {
    //pub fn new(image_handle: Handle<Image>) -> Self { }
}

#[derive(Event, Debug)]
pub struct ClearBelow {
    pub y_cutoff: f32,
}

/// Wrapper for `image::Image`. Expects `TextureFormat::Rgba8Uint`,
/// meaning that each pixel is RGBA and has four `u8`s.
pub struct Pixels<'a> {
    width: f32,
    image: &'a mut Image,
}

impl<'a> Pixels<'a> {
    /// Creates new `Pixels` from `Image`. Panics if `image` does not of
    /// `TextureFormat::Rgba8Uint`.
    pub fn new(image: &'a mut Image) -> Pixels<'a> {
        dbg!(image.texture_descriptor.format);
        assert!(image.texture_descriptor.format == TextureFormat::Rgba8Unorm);
        Pixels {
            width: image.size().as_vec2().x,
            image,
        }
    }

    pub fn iter_mut(&'a mut self) -> EnumeratePixelsMut<'a> {
        EnumeratePixelsMut::new(
            self.width as u32,
            self.image.data.rchunks_exact_mut(4),
        )
    }

    // Clear pixels below y_cutoff. y_cutoff is expected to be in bevy's global
    // coordinate space
    pub fn clear_below(
        &mut self,
        y_cutoff: u32,
        global_transform: &GlobalTransform,
    ) {
        //let x = self.into_iter();
    }
}

impl<'a> IntoIterator for Pixels<'a> {
    type Item = (u32, u32, &'a mut [u8]);
    type IntoIter = EnumeratePixelsMut<'a>;

    fn into_iter(self) -> Self::IntoIter {
        EnumeratePixelsMut::new(
            self.width as u32,
            self.image.data.rchunks_exact_mut(4),
        )
    }
}

impl<'a> IntoIterator for &'a mut Pixels<'a> {
    type Item = (u32, u32, &'a mut [u8]);
    type IntoIter = EnumeratePixelsMut<'a>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter_mut()
    }
}

pub struct EnumeratePixelsMut<'a> {
    width: u32,
    x: u32,
    y: u32,
    /// We want this to be `&mut [u8; 4]`
    pixels: RChunksExactMut<'a, u8>,
}

impl<'a> EnumeratePixelsMut<'a> {
    pub fn new(width: u32, pixels: RChunksExactMut<'a, u8>) -> Self {
        Self {
            width,
            x: 0,
            y: 0,
            pixels,
        }
    }
}

/// Pillaged from `image::buffer::EnumeratePixelsMut`
impl<'a> Iterator for EnumeratePixelsMut<'a> {
    /// (x, y, chunk). `chunk: &'a mut u8` is one RGBA value made up of four
    /// `u8`s.
    type Item = (u32, u32, &'a mut [u8]);

    fn next(&mut self) -> Option<Self::Item> {
        if self.x >= self.width {
            self.x = 0;
            self.y += 1;
        }
        let (x, y) = (self.x, self.y);
        self.x += 1;
        self.pixels.next().map(|p| (x, y, p))
    }
}

#[derive(Debug)]
pub enum RgbaChannel {
    Red,
    Green,
    Blue,
    Alpha,
}

trait RgbaPixel {
    fn raw_mut(&mut self) -> &mut [u8];

    fn raw(&self) -> &[u8];

    fn get_channel(&self, channel: RgbaChannel) -> u8 {
        match channel {
            RgbaChannel::Red => self.raw()[0],
            RgbaChannel::Green => self.raw()[1],
            RgbaChannel::Blue => self.raw()[2],
            RgbaChannel::Alpha => self.raw()[3],
        }
    }

    fn get_channel_mut(&mut self, channel: RgbaChannel) -> &mut u8 {
        match channel {
            RgbaChannel::Red => &mut self.raw_mut()[0],
            RgbaChannel::Green => &mut self.raw_mut()[1],
            RgbaChannel::Blue => &mut self.raw_mut()[2],
            RgbaChannel::Alpha => &mut self.raw_mut()[3],
        }
    }
}

/// Expects the mutable slice to have four `u8`s.
impl RgbaPixel for &mut [u8] {
    fn raw_mut(&mut self) -> &mut [u8] {
        self
    }

    fn raw(&self) -> &[u8] {
        self
    }
}

// TODO: repllace with `Trigger<SliceImage>`
//
// manually edit resource on slice
pub fn clear_below(
    trigger: Trigger<ClearBelow>,
    //mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    mut square: Query<
        (Entity, &mut Handle<Image>, &GlobalTransform),
        With<SquareImage>,
    >,
) {
    let ClearBelow { y_cutoff } = trigger.event();
    info!("{:?}", trigger.event());
    if let Ok((entity, mut image_handle, global_transform)) =
        square.get_single_mut()
    {
        // NOTE: rounding from cast could be an issue
        dbg!(y_cutoff);

        if let Some(mut image) = images.remove(image_handle.as_mut()) {
            let mut pixels = Pixels::new(&mut image);
            for (x, y, mut pixel) in pixels {
                let local_p = Vec3::new(x as f32, y as f32, 0.0);
                let global_p = global_transform.transform_point(local_p);
                if global_p.y < *y_cutoff {
                    *pixel.get_channel_mut(RgbaChannel::Alpha) = 0;
                }
            }

            *image_handle = images.add(image);
        }
    }
}

#[derive(Event, Debug)]
pub struct SliceImage {
    pub sprite_id: Entity,
    pub rows_to_slice: Vec<u8>,
}

/// NOTE: this must be scheduled /after/ the sprite being sliced has been added
/// to a rigid body or the global transform will be wrong (it could take a
/// frame to propagate, so expect some fiddling)
pub fn apply_slice_image(
    mut slices: EventReader<SliceImage>,
    mut images: ResMut<Assets<Image>>,
    mut sprite: Query<
        (Entity, &mut Handle<Image>, &GlobalTransform),
        With<SquareImage>,
    >,
) {
    // apply row bounds to pixels in one pass-- see `clear_below`

    for s @ SliceImage {
        sprite_id,
        rows_to_slice,
    } in slices.read()
    {
        info!("{:?}", s);
        let Ok((entity, mut image_handle, global_transform)) =
            sprite.get(*sprite_id)
        else {
            warn!("apply_slice_image: sprite_id, {:?}, not found", &sprite_id);
            break;
        };

        let Some(mut image) = images.get_mut(image_handle) else {
            warn!("");
            break;
        };

        info!("slicing sprite: {:?}", sprite_id);

        let mut pixels = Pixels::new(image);
        for (x, y, mut pixel) in pixels.iter_mut() {
            let local_p = Vec3::new(x as f32, y as f32, 0.0);
            let global_p = global_transform.transform_point(local_p);
            //if in_row(global_p)
            if rows_to_slice
                .iter()
                .filter_map(|&row| RowBounds::new(row))
                .any(|bounds| bounds.contains_vec3(global_p))
            {
                *pixel.get_channel_mut(RgbaChannel::Alpha) = 0;
            }
        }
    }
}
