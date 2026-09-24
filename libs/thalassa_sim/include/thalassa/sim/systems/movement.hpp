#pragma once

#include "thalassa/core/ecs/world.hpp"
#include "thalassa/core/types.hpp"

// Movement systems: straight-line "deterministic path
// following" (no obstacle avoidance or full
// pathfinding yet, that's later). Two systems, run in this
// fixed order every tick (see thalassa::sim::SimWorld::tick):
//
//   1. steering_system: for every entity with (Position, Destination),
//      sets Velocity to point toward the destination at the entity's
//      configured speed, or — if this tick's travel distance
//      (speed * dt) would reach or pass the destination — snaps Position
//      directly to the target, zeroes Velocity, and removes Destination.
//      That snap-on-final-step is not an optimization, it's a correctness
//      requirement: without it, an entity whose per-tick step is larger
//      than arrival_radius (the common case — a small arrival_radius like
//      0.1 units is much smaller than a typical multi-unit-per-tick step)
//      would overshoot the target every tick, flip direction, overshoot
//      back, and oscillate around the destination forever without ever
//      landing inside arrival_radius.
//   2. integration_system: for every entity with (Position, Velocity),
//      applies `position += velocity * dt`. Runs over ALL entities with
//      Velocity, not just ones with an active Destination, so future
//      systems (knockback, projectile motion) can reuse plain
//      Position+Velocity integration without going through Destination at
//      all.
//
// Splitting steering from integration (rather than one combined system)
// keeps each function's determinism argument simple to state in isolation
// and means integration_system alone is reusable for any future
// Velocity-driven motion.

namespace thalassa::sim::systems {

void steering_system(core::ecs::World& world, core::Scalar dt_seconds);
void integration_system(core::ecs::World& world, core::Scalar dt_seconds);

}  // namespace thalassa::sim::systems
