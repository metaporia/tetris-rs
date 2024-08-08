//! # Image demo
//!
//! - Load in `Tetromino` square images
//! - attempt software pixel discard
//! - otherwise write simple fragment shader with custom material
//! - A third option would be a vertex shader (if it works in 2d) and adjust
//!   the clipping coordinates to exclude slices.

use std::fs::File;
use std::slice::{ChunksExactMut, RChunksExactMut};

use bevy::math::VectorSpace;
use bevy::render::render_resource::TextureFormat;
use bevy::sprite::Anchor;
use bevy::utils::HashMap;
use bevy::{prelude::*, render::render_asset::RenderAssetUsages};
use bevy_inspector_egui::inspector_egui_impls::register_std_impls;
use bevy_rapier2d::prelude::*;
use image::imageops::FilterType;
use image::{DynamicImage, GenericImageView, ImageBuffer, Pixel, RgbaImage};
use std::ops::Deref;

use crate::game::{transform_point, RowBounds, SliceRows, GROUND_Y};
use crate::tetroid::*;
use crate::BRICK_DIM;
use crate::{draw_circle_contact, ActiveTetromino};

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
    image_handle_to_sprite_bundle(image_handle)
}

/// Add image to `Assets<Image>`, create new sprite bundle with the resultant
/// image handle.
pub fn image_handle_to_sprite_bundle(
    image_handle: Handle<Image>,
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

/// Takes the tetromino type and gets the associated asset.
///
/// Since we don't want to load block assets from the filesystem every time a
/// `Tetromino` is spawned, let's load reference handles that don't get used,
/// and are simple available to copy.
///
/// There is an asset (30x30 pixel square) indexed by `TetrominoType`.
///
/// But--and it's big--the assets from nottetris not split by block. Load
/// order:
/// - init: load tetromino assets (one png per tetromino body), slice and dice,
///   & add the four resultant image handles (`Vec<Handle<Image>>`) into a
///   `type TetrominoImageMap = Res<HashMap<TetrominoType, Vec<Handle<Image>>>`
/// - spawn_new_tetromino: fetches all four assets, clones them, inserts the
///   handles into the asset map, and returns a list of bundles.
///   NOTE: order matters here--we could index the colliders and assets slices
///   to ensure they get matched properly
pub fn new_tetromino_handles(
    mut images: Assets<Image>,
    block_type: TetrominoType,
) -> Vec<Handle<Image>> {
    todo!();
}

// PLUGIN START
pub struct TetrominoAssetPlugin;

#[derive(Debug, Resource, Default, Deref, DerefMut)]
pub struct TetrominoAssetMap(HashMap<TetrominoType, Vec<Handle<Image>>>);

//impl Deref for TetrominoAssetMap {
//    type Target = HashMap<TetrominoType, Vec<Handle<Image>>>;
//
//    fn deref(&self) -> &Self::Target {
//        self.
//    }
//}

impl Plugin for TetrominoAssetPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(TetrominoAssetMap::default())
            .add_systems(PreStartup, load_block_assets);
    }
}

/// Demo: load png for I-block into `TetrominoAssetMwp`.
///
/// Must be scheduled after `TetrominoAssetMap` initialization.
pub fn load_block_assets(
    mut asset_map: ResMut<TetrominoAssetMap>,
    mut images: ResMut<Assets<Image>>,
) {
    // TODO: special handling for I-block as it has three distinct blocks:
    // left, middle (2x), and right squares,
    for i in 1..8 {
        let block_type = TetrominoType::from_idx(i).unwrap();
        let file_name =
            format!("assets/scaled/{:?}-square.png", block_type.as_idx());
        dbg!(&file_name);
        let scaled = image::open(file_name).unwrap().resize_exact(
            BRICK_DIM as u32,
            BRICK_DIM as u32,
            FilterType::Nearest,
        );

        //let mut output = File::create("scaled.png").unwrap();
        //scaled.write_to(&mut output, image::ImageFormat::Png);
        dbg!(scaled.height(), scaled.width());

        let scaled_handle = images.add(Image::from_dynamic(
            scaled,
            false,
            RenderAssetUsages::RENDER_WORLD | RenderAssetUsages::MAIN_WORLD,
        ));
        asset_map.insert(block_type, vec![scaled_handle]);
    }
}

/// Clone reference block image from asset_map and return sprite bundle
/// containing image handle referencing the cloned copy.
pub fn tetromino_type_to_sprite_bundle(
    block_type: &TetrominoType,
    mut images: &mut Assets<Image>,
    asset_map: &TetrominoAssetMap,
) -> (SquareImage, SpriteBundle) {
    //dbg!(&block_type);
    //dbg!(&asset_map);
    let handles = asset_map.get(block_type).unwrap();
    let i_block_handle = handles[0].clone();
    //dbg!(&i_block_handle);
    let new_image = images.get(&i_block_handle).unwrap().clone();
    let new_handle = images.add(new_image);
    image_handle_to_sprite_bundle(new_handle)
}

// PLUGIN END

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
        //dbg!(image.texture_descriptor.format);
        assert!(image.texture_descriptor.format == TextureFormat::Rgba8Unorm);
        Pixels {
            width: image.size().as_vec2().x,
            image,
        }
    }

    /// Index raw image data pixel quartets.
    pub fn get(&self, idx: usize) -> &[u8] {
        let start = idx * 4;
        let end = start + 4;
        &self.image.data[start..end]
    }

    /// Index raw image data pixel quartets.
    pub fn get_mut(&mut self, idx: usize) -> &mut [u8] {
        let start = idx * 4;
        let end = start + 4;
        &mut self.image.data[start..end]
    }

    pub fn dims(&self) -> Vec2 {
        self.image.size().as_vec2()
    }

    pub fn iter_mut(&'a mut self) -> EnumeratePixelsMut<'a> {
        EnumeratePixelsMut::new(
            self.width as u32,
            self.image.data.chunks_exact_mut(4),
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
            self.image.data.chunks_exact_mut(4),
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
    pixels: ChunksExactMut<'a, u8>,
}

impl<'a> EnumeratePixelsMut<'a> {
    pub fn new(width: u32, pixels: ChunksExactMut<'a, u8>) -> Self {
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
    pub slice_rows: SliceRows,
    /// The bounding x-coordinates of the sprite's parent collider in global
    /// space. `(leftmost: f32, rightmost: f32)`
    pub x_bounds: (f32, f32),
    /// The bounding y-coordinates of the sprite's parent collider in global
    /// space.`(lowest: f32, highest: f32)`
    pub y_bounds: (f32, f32),
    /// The global transform of the old sprite bundle 
    pub global_transform: GlobalTransform,
}

impl SliceImage {
    /// Takes a Vec3 in global space and returns whether it is within the
    /// sprite's parent collider's x- and y-bounds.
    pub fn is_point_in_bounds(&self, p: Vec3) -> bool {
        let (left, right) = self.x_bounds;
        let (low, high) = self.y_bounds;
        left <= p.x && p.x <= right && low <= p.y && p.y <= high
    }
}

/// Slices image with row bounds and trims stray pixels (with contiguity check, I guess)
///
/// Trim logic:
/// - pass collider's left- and right-most x and lowest and higest y, and trim
///   all pixels outside of the bounding box.
pub fn apply_slice_image(
    mut slices: EventReader<SliceImage>,
    mut images: ResMut<Assets<Image>>,
    mut sprite: Query<
        (Entity, &mut Handle<Image>),
        With<SquareImage>,
    >,
    mut cmds: Commands,
) {
    // apply row bounds to pixels in one pass-- see `clear_below`

    for s @ SliceImage {
        sprite_id,
        slice_rows,
        x_bounds,
        y_bounds,
        global_transform
    } in slices.read()
    {
        let Ok((entity, mut image_handle)) =
            sprite.get(*sprite_id)
        else {
            error!(
                "apply_slice_image: sprite_id, {:?}, not found",
                &sprite_id
            );
            break;
        };

        let Some(mut image) = images.get_mut(image_handle) else {
            warn!("Failed to get image handle");
            break;
        };
        once!(info!("Slicing images"));
        //info!("slicing sprite: {:?}", sprite_id);
        let origin = Vec2::ZERO;

        let mut pixels = Pixels::new(image);
        let height = pixels.width;
        for (x, y, mut pixel) in pixels.iter_mut() {
            let local_p = Vec3::new(x as f32, height - y as f32, 0.0);
            let global_p = global_transform.transform_point(local_p);
            // discard pixel if it's in a slice row or beyond the slice's x- &
            // y-bounds
            if slice_rows
                .rows
                .iter()
                .filter_map(|&row| RowBounds::new(row))
                .any(|bounds| bounds.contains_vec3(global_p))
                || !s.is_point_in_bounds(global_p)
            {
                *pixel.get_channel_mut(RgbaChannel::Alpha) = 0;
            }
        }
    }
}
