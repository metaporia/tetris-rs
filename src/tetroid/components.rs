use bevy::ecs::component::Component;

/* Single Block plugin */
/// Nominal component used to label actively falling tetroid.
#[derive(Component)]
pub struct ActiveTetroid;

#[derive(Component)]
pub struct LBlock;

/// Tetroids are compound shapes made up of convex polygons
#[derive(Component)]
pub struct ActiveTetroidComponent;
