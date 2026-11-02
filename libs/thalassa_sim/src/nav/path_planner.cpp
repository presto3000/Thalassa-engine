#include "thalassa/sim/nav/path_planner.hpp"

#include <algorithm>

#include "thalassa/sim/components.hpp"

namespace thalassa::sim::nav {

using components::Destination;
using components::Path;
using components::Position;
using components::Radius;
using components::Velocity;

PathPlanner::PathPlanner(std::shared_ptr<const NavMap> map) : map_(std::move(map)) {
    finders_.resize(map_->class_count());
}

void PathPlanner::plan(core::ecs::World& world, core::ecs::Entity entity) {
    if (!world.is_alive(entity) || !world.has<Position>(entity) || !world.has<Destination>(entity)) return;

    const core::math::Vec3 from = world.get<Position>(entity).value;
    const Destination dest = world.get<Destination>(entity);
    const core::Scalar radius = world.has<Radius>(entity) ? world.get<Radius>(entity).value : 0.0;
    const std::size_t cls = map_->class_for_radius(radius);

    if (!finders_[cls]) finders_[cls] = std::make_unique<Pathfinder>(map_->grid(cls));
    ++searches_;
    const PathResult result = finders_[cls]->find_path(from, dest.target);

    if (result.status == PathStatus::NoPath || result.waypoints.empty()) {
        // Nowhere to go: forget the order.
        world.remove<Destination>(entity);
        if (world.has<Path>(entity)) world.remove<Path>(entity);
        world.emplace<Velocity>(entity, Velocity{core::math::Vec3{0.0, 0.0, 0.0}});
        return;
    }

    Path path;
    const std::size_t n = std::min<std::size_t>(result.waypoints.size(), Path::kMaxWaypoints);
    for (std::size_t i = 0; i < n; ++i) path.points[i] = result.waypoints[i];
    path.count = static_cast<std::uint16_t>(n);
    path.next = 0;
    if (result.status == PathStatus::Partial) path.flags |= Path::kPartial;
    if (result.waypoints.size() > n) path.flags |= Path::kTruncated;
    world.emplace<Path>(entity, path);
}

}  // namespace thalassa::sim::nav
