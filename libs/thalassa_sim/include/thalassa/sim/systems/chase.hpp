#pragma once

#include "thalassa/core/ecs/world.hpp"
#include "thalassa/core/types.hpp"

namespace thalassa::sim::nav {
class PathPlanner;
}

// Combat uses paths (pathfinding step 4b): units with components::Chase (from
// Command::AttackTarget) go after their target, ranged and melee alike.
//
// Every tick, for each chaser (component-pool order):
//   - target gone (dead, destroyed) -> the order ends: Chase, Destination and Path are removed
//     and the unit stops.
//   - target within reach (kReachFraction of the unit's attack_range, so it stops slightly
//     inside range and does not jitter on the edge) -> stop walking; combat_system fights, and
//     prefers this target over other enemies in range.
//   - otherwise walk toward it: set a Destination at the target's position and route it with
//     the planner (straight line without a nav map). The route is REPLANNED when the target has
//     moved more than kReplanDistance from where the route ends, at most every
//     Chase::kMinReplanTicks ticks, so chasing a moving enemy costs a bounded number of searches.
// A melee unit (small attack_range) therefore closes to touching distance, a ranged one stops at
// its range: "ranged repositioning" is the same rule with a bigger number.
// An order that stalls in a crowd (crowd_system gives up) is simply re-issued after the replan delay.
//
// Runs after commands are applied and before steering_system. `planner` may be null.

namespace thalassa::sim::systems {

inline constexpr core::Scalar kChaseReachFraction = 0.9;
inline constexpr core::Scalar kChaseReplanDistance = 1.0;  // meters

void chase_system(core::ecs::World& world, nav::PathPlanner* planner);

}  // namespace thalassa::sim::systems
