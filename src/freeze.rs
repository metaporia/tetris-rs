use bevy::{
    app::{App, Plugin},
    prelude::*,
    time::{Timer, TimerMode},
};
use bevy_rapier2d::prelude::*;
use tetris_rs::GameState;

use crate::game::{FallSpeed, Tetroid};

/// Expects GameState to already have been initialized.
pub fn plugin(app: &mut App) {
    // TODO: add `blink_row` that gets the current SliceRows
    // and blinks `em
    app.insert_resource(FreezeTimer::new())
        .add_systems(
            Update,
            FreezeTimer::tick.run_if(in_state(GameState::Frozen)),
        )
        .add_systems(OnEnter(GameState::Frozen), (FreezeTimer::init, freeze))
        .add_systems(OnExit(GameState::Frozen), unfreeze);
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

/// On ground contact remove all velocity & gravity for ActiveTetroid, remove
/// ActiveTetroid from entity
pub fn freeze(
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
) {
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

/// TODO: unfreeze with some momentum
pub fn unfreeze(
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
    //mut unfreeze: EventReader<UnFreeze>,
    mut fallspeed: Res<FallSpeed>,
) {
    info!("Unfreezing");
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
        gravity.0 = 2.0;
        external_force.force = Vec2::ZERO;
        external_force.torque = 0.0;
        damping.linear_damping = 0.0;
    }
}
