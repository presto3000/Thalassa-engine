#pragma once

#include <memory>
#include <vector>

#include "thalassa/core/ecs/world.hpp"
#include "thalassa/sim/nav/nav_map.hpp"
#include "thalassa/sim/nav/pathfinder.hpp"

// PathPlanner: turns "this entity wants to go to its Destination" into a components::Path
// on the shared NavMap. One per SimWorld (it owns the reusable search scratch memory).
//
// plan() looks at the entity's Position, Destination and (optional) Radius, routes it on
// the matching size class, and writes the Path. Outcomes:
//   - route found / partial: Path set (partial routes end at the closest reachable point)
//   - no movement possible (enclosed, off the map): the order is dropped (Destination and
//     Path removed, velocity zeroed), exactly as if the unit had arrived
// Every query runs immediately; a per-tick budget/queue for big groups is step 5 of the plan.

namespace thalassa::sim::nav {

class PathPlanner {
public:
    explicit PathPlanner(std::shared_ptr<const NavMap> map);

    void plan(core::ecs::World& world, core::ecs::Entity entity);

    [[nodiscard]] const NavMap& map() const noexcept { return *map_; }
    [[nodiscard]] std::uint64_t searches() const noexcept { return searches_; }  // diagnostics

private:
    std::shared_ptr<const NavMap> map_;
    std::vector<std::unique_ptr<Pathfinder>> finders_;  // one per size class, created on first use
    std::uint64_t searches_ = 0;
};

}  // namespace thalassa::sim::nav
