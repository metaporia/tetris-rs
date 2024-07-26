//! # Image demo
//!
//! - Load in `Tetromino` square images
//! - attempt software pixel discard
//! - otherwise write simple fragment shader with custom material
//! - A third option would be a vertex shader (if it works in 2d) and adjust
//!   the clipping coordinates to exclude slices.

use bevy::prelude::*;

use crate::tetroid::*;

use image::{DynamicImage, ImageBuffer, Pixel};

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

/// Generate a `crate::tetroid::TetrominoType::SQUARE`-sized image filled with
/// blue.
pub fn make_image() {
    let mut image = ImageBuffer::new(BRICK_DIM as u32, BRICK_DIM as u32);
    for (x, y, pixel) in image.enumerate_pixels_mut() {
        let p = image::Rgba::<u8>([0, 255, 255, 200]);
        *pixel = p;
    }

    clear_pixels_below_y(15, &mut image);
    image.save("blue_square.png").unwrap();
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

//fn setup_image
