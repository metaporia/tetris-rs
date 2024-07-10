# tetris-rs

A [Not Tetris 2](https://stabyourself.net/nottetris2/) clone in rust.

## Why?

Because Love 0.7 is being deprecated all over the place, performance is not
great, and something about idle hands.


## Development Notes


### Deps

Just follow bevy's 
[installation instructions](https://github.com/bevyengine/bevy/blob/main/docs/linux_dependencies.md)
or use the tetris-rs flake. Currently only the flake's shell is functional.

### Libs

- [bevy](https://bevyengine.org/) game engine
- [rapier](https:://rapier.rs) bevy plugin for physics
- [bevy_prototype_lyon](https://github.com/Nilirad/bevy_prototype_lyon) for
  drawing simple shapes


### System Order

StartGame -> SetupArena -> SpawnActiveTetroid 
- on_tick/Update: key_handling, calculate_row_density (all rows) -> SliceRow(Row)
- *HitDebrisOrGround -> FreezePhysics -> CalculateRowDensity [-> SliceRow(Row)] -> SpawnActiveTetroid
- SliceRow(Row) 

- on HitDebrisOrGround: 
    - remove Active component from ActiveTetroid entity and all of its
      children
    - ch


