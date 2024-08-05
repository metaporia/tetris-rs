//! Provides app states, and system sets
//!
//! I think we can move the core update loop into plugins and then use system
//! sets to order them
//!

use bevy::prelude::*;

#[derive(States, Default, Debug, Clone, PartialEq, Eq, Hash)]
pub enum AppState {
    LoadingMenu,
    #[default]
    MainMenu,
    /// Spawn arena, load assets, happen here
    /// Also spawn first tetromino and start the game simultaneously.
    InitialGameSetup,
    InGame,
}

#[derive(SubStates, Default, Debug, Clone, PartialEq, Eq, Hash)]
#[source(AppState = AppState::InGame)]
pub enum PausedState {
    Paused,
    #[default]
    Playing,
}

#[derive(SubStates, Default, Debug, Clone, PartialEq, Eq, Hash)]
#[source(PausedState = PausedState::Playing)]
pub enum GameState {
    #[default]
    Playing,
    Frozen,
}

pub const ROW_DENSITY_THRESHOLD: f32 = 0.7;
