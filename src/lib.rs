//! Provides app states, and system sets
//!
//! I think we can move the core update loop into plugins and then use system
//! sets to order them
//!

pub enum AppState {
    LoadingScreen,
    MainMenu,
    InGame,
}

pub const ROW_DENSITY_THRESHOLD: f32 = 0.7;
