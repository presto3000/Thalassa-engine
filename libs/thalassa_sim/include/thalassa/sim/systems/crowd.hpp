#pragma once

#include "thalassa/core/ecs/world.hpp"
#include "thalassa/core/types.hpp"
#include "thalassa/sim/nav/nav_map.hpp"
#include "thalassa/sim/spatial_index.hpp"

// Units vs units (pathfinding step 4a): soft collision plus "give up when stuck".
//
// Only entities with a Radius > 0 take part (the same component that picks the nav size class),
// so worlds without it behave exactly as before.
//
// 1. Separation. Two circles that overlap are pushed apart along the line between their centers.
//    Moving units (with a Destination) yield more than idle ones (idle_weight), so a column
//    flows around a resting unit instead of shoving it across the map. Each unit resolves only
//    a fraction of its overlap per tick and never moves more than max_push_per_tick, so crowds
//    relax smoothly instead of exploding. A push that would put a unit into a blocked cell of its
//    size class is dropped (tried per axis first), so crowds never squeeze anyone into a wall.
// 2. Stall detection (MoveProgress). A unit with an order that cannot make headway (its goal is
//    occupied, it is jammed in a crowd) ends the order after a few stalled windows and rests
//    where it is; near the goal quickly, far from it only after much longer.
//
// Determinism: displacements are computed for every unit from the SAME start-of-tick positions
// (Jacobi style, nothing is written while reading), each unit sums its own neighbours in the
// spatial index's fixed order, and the results are applied in component-pool order. A pair at
// exactly the same position is split along +x/-x by entity index. Nothing depends on hash order
// or on which unit happens to be processed first.

namespace thalassa::sim::systems {

struct CrowdParams {
    core::Scalar idle_weight = 0.25;        // an idle unit yields this much compared to a moving one (weight 1)
    core::Scalar push_fraction = 0.5;       // share of the overlap a unit resolves per tick
    core::Scalar max_push_per_tick = 0.1;   // meters; also caps the total push from many neighbours
    std::uint32_t stalled_windows_near = 1;  // windows without progress before giving up close to the goal
    std::uint32_t stalled_windows_far = 4;   // ... or far from it
    core::Scalar near_goal_distance = 3.0;   // "close to the goal" (meters), at least 4 radii
    core::Scalar min_progress_fraction = 0.25;  // a window needs this share of the distance the speed allows
};

// Returns true if any unit's position changed (the caller then rebuilds the spatial index).
// `index` must be current for the positions in `world`. `map` may be null (no wall check).
bool crowd_system(core::ecs::World& world, const SpatialIndex& index, const nav::NavMap* map, core::Scalar dt_seconds,
                  const CrowdParams& params = {});

}  // namespace thalassa::sim::systems
