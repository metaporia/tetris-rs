use crate::{ActiveTetromino, Pause, IMPULSE_SCALAR};
use bevy::prelude::*;
use bevy_rapier2d::dynamics::{
    Damping, ExternalForce, ExternalImpulse, Velocity,
};
use tetris_rs::{AppState, GameState, PausedState};

pub fn plugin(mut app: &mut App) {
    app.add_event::<Pause>().add_systems(
        Update,
        (
            kbd_input.run_if(in_state(GameState::Playing)),
            (get_pause_input, toggle_pause).run_if(in_state(AppState::InGame)),
        ),
    );
}

/// Toggle the pouse state and virtual time.
pub fn toggle_pause(
    mut time: ResMut<Time<Virtual>>,
    mut pause: EventReader<Pause>,
    mut next_state: ResMut<NextState<PausedState>>,
) {
    for p in pause.read() {
        if time.is_paused() {
            time.unpause();
            next_state.set(PausedState::Playing);
        } else {
            time.pause();
            next_state.set(PausedState::Paused);
        }
    }
}

/// Handle pause input in isolation in order to disable keyboard input while
/// frozen.
pub fn get_pause_input(
    kbd_input: Res<ButtonInput<KeyCode>>,
    mut pause: EventWriter<Pause>,
) {
    if kbd_input.just_pressed(KeyCode::Escape) {
        pause.send(Pause);
    }
}

/// Handle all keyboard input but Esc (which is handled by `get_pause_input`).
pub fn kbd_input(
    kbd_input: Res<ButtonInput<KeyCode>>,
    mut ext_impulses: Query<
        (
            &mut ExternalImpulse,
            &mut ExternalForce,
            &mut Velocity,
            &mut Damping,
        ),
        With<ActiveTetromino>,
    >,
    //active_tetroid_query: Query<Entity, With<ActiveTetroid>>,
) {
    let Ok((mut ext_impulse, mut ext_force, mut velocity, mut damping)) =
        ext_impulses.get_single_mut()
    else {
        return;
    };

    let force = 2000000.0;
    let torque = force * 15.0;
    let max_vel = 230.0;
    let max_ang_vel = 2.7;

    let mut torque_impulse = 0.0;
    let torque_imp = 13.0 * IMPULSE_SCALAR;
    if kbd_input.pressed(KeyCode::KeyZ) && velocity.angvel < max_ang_vel {
        ext_force.torque = torque;
        //velocity.angvel = -max_ang_vel;
    }
    //
    //
    // - put `ColliderMassProperties` on rigibbody
    // - remove `ColliderMassProperties` from colliders
    // - ideally we'd like them all to rotate identicially, so identical
    //   angular inertia & center of mass. Hopefully a point mass will do this.

    // - NB: remove all forces on `DeactivateTetroid`
    else if kbd_input.pressed(KeyCode::KeyX)
        && velocity.angvel > -max_ang_vel
    {
        ext_force.torque = -torque;
        //velocity.angvel = max_ang_vel;
    } else {
        ext_force.torque = 0.0;
    }

    let mut impulse = Vec2::new(0.0, 0.0);
    let lateral_imp = 3.0 * IMPULSE_SCALAR;
    let vertical_imp = lateral_imp;

    // cap angular and linear velocities
    // otherwise apply impulses
    if kbd_input.pressed(KeyCode::ArrowLeft) && velocity.linvel.x > -max_vel {
        ext_force.force.x = -force;
    } else if kbd_input.pressed(KeyCode::ArrowRight)
        && velocity.linvel.x < max_vel
    {
        ext_force.force.x = force;
    } else {
        ext_force.force.x = 0.0;
    }

    let max_boost_vel = -300.0;
    if kbd_input.pressed(KeyCode::ArrowDown) {
        if velocity.linvel.y > max_boost_vel {
            ext_force.force.y = -force
        }
    } else {
        ext_force.force.y = 0.0;
        // apply damping if above speed limit.
        if velocity.linvel.y < max_boost_vel {
            ext_force.force.y = force;
        } else {
            ext_force.force.y = 0.0;
        }
        // slow down block until it reaches 250.
    }
}
