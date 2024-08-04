//! Provides app states, and system sets
//!
//! I think we can move the core update loop into plugins and then use system
//! sets to order them
//!

use bevy::prelude::*;

#[derive(States, Default, Debug, Clone, PartialEq, Eq, Hash)]
pub enum AppState {
    LoadingScreen,
    #[default]
    MainMenu,
    InGame,
}

#[derive(SubStates, Default, Debug, Clone, PartialEq, Eq, Hash)]
#[source(AppState = AppState::InGame)]
pub enum GameState {
    #[default]
    Playing,
    Frozen,
    Paused,
}

pub const ROW_DENSITY_THRESHOLD: f32 = 0.7;

/// Expects GameState to already have been initialized.
///
pub struct FreezePlugin;

impl Plugin for FreezePlugin {
    fn build(&self, app: &mut App) {
        // TODO: add `blink_row` that gets the current SliceRows
        // and blinks `em
        app.insert_resource(FreezeTimer::new())
            .add_systems(OnEnter(GameState::Frozen), FreezeTimer::init)
            .add_systems(
                Update,
                FreezeTimer::tick.run_if(in_state(GameState::Frozen)),
            );
    }
}

// START Freeze logic
#[derive(Resource)]
struct FreezeTimer(Timer);

impl FreezeTimer {
    pub fn new() -> Self {
        FreezeTimer(Timer::from_seconds(0.6, TimerMode::Repeating))
    }

    // frozen state time
    // - we could instead put the timer in the state enum so it only exists in the
    //   `Frozen` state
    fn init(mut timer: ResMut<FreezeTimer>) {
        // reset timer
        timer.0.reset();
        timer.0.unpause();
    }

    // tick timer until it finishes and then exit state
    fn tick(
        time: Res<Time>,
        mut timer: ResMut<FreezeTimer>,
        mut next_state: ResMut<NextState<GameState>>,
    ) {
        if timer.0.tick(time.delta()).just_finished() {
            dbg!(timer.0.remaining());
            next_state.set(GameState::Playing);
        }
    }
}

