#pragma once

#include "thalassa/core/ecs/world.hpp"
#include "thalassa/core/types.hpp"
#include "thalassa/sim/spatial_index.hpp"

// Combat systems. Run in this fixed order, after movement
// has been resolved for the tick (see SimWorld::tick):
//
//   1. combat_system: for every entity with (Position, Team, CombatStats),
//      ticks its cooldown down; if ready and an enemy is within
//      attack_range (found via the spatial index, nearest-wins, ties
//      broken by lowest Entity::index, spawns a Projectile entity
//      aimed at it and resets the cooldown.
//   2. projectile_collision_system: for every entity with (Position,
//      Projectile), checks for an enemy within a fixed hit radius of its
//      current position; on hit, applies damage and destroys the
//      projectile, otherwise ages it down by dt and destroys it if its
//      ttl has expired.
//   3. death_system: destroys every entity whose Health.current <= 0.
//
// All three take the SpatialIndex by const reference rather than
// rebuilding their own — SimWorld::tick() rebuilds it once, right after
// movement, and every system that needs "who's near whom" this tick reads
// that same snapshot, keeping "what does the world look like this tick"
// unambiguous across combat_system and projectile_collision_system.

namespace thalassa::sim::systems {

void combat_system(core::ecs::World& world, const SpatialIndex& spatial_index, core::Scalar dt_seconds);
void projectile_collision_system(core::ecs::World& world, const SpatialIndex& spatial_index, core::Scalar dt_seconds);
void death_system(core::ecs::World& world);

}  // namespace thalassa::sim::systems
