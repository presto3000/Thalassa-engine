#include "thalassa/sim/systems/crowd.hpp"

#include <vector>

#include "thalassa/core/math/strictfp.hpp"
#include "thalassa/core/math/vector.hpp"
#include "thalassa/sim/components.hpp"

namespace thalassa::sim::systems {

using core::Scalar;
using core::ecs::Entity;
using core::math::Vec3;
using components::Destination;
using components::MoveProgress;
using components::Path;
using components::Position;
using components::Radius;
using components::Velocity;

namespace {

struct Agent {
    Entity entity;
    Vec3 position;
    Scalar radius = 0.0;
    bool moving = false;
};

bool blocked_at(const nav::NavMap* map, Scalar radius, const Vec3& p) {
    if (map == nullptr) return false;
    const nav::NavGrid& grid = map->grid(map->class_for_radius(radius));
    const auto cell = grid.to_cell(p);
    return !cell || grid.blocked(*cell);
}

}  // namespace

bool crowd_system(core::ecs::World& world, const SpatialIndex& index, const nav::NavMap* map, Scalar dt_seconds,
                  const CrowdParams& params) {
    // Pass 0: the agents, in pool order.
    std::vector<Agent> agents;
    Scalar max_radius = 0.0;
    world.each<Position, Radius>([&](Entity e, Position& pos, Radius& radius) {
        if (!(radius.value > 0.0)) return;
        agents.push_back(Agent{e, pos.value, radius.value, world.has<Destination>(e)});
        if (radius.value > max_radius) max_radius = radius.value;
    });
    if (agents.empty()) return false;

    // Pass 1: displacement per agent from start-of-tick positions (read only).
    std::vector<Vec3> push(agents.size(), Vec3{0.0, 0.0, 0.0});
    std::vector<Entity> near;
    bool any_push = false;
    for (std::size_t i = 0; i < agents.size(); ++i) {
        const Agent& a = agents[i];
        near.clear();
        index.query_radius(a.position, a.radius + max_radius, near);
        const Scalar weight_a = a.moving ? 1.0 : params.idle_weight;
        Vec3 total{0.0, 0.0, 0.0};
        for (const Entity other : near) {
            if (other == a.entity) continue;
            if (!world.has<Radius>(other)) continue;
            const Scalar other_radius = world.get<Radius>(other).value;
            if (!(other_radius > 0.0)) continue;
            const Vec3 other_pos = world.get<Position>(other).value;  // unchanged so far: nothing is written in this pass
            const Scalar weight_b = world.has<Destination>(other) ? 1.0 : params.idle_weight;

            const Scalar dx = a.position.x - other_pos.x;
            const Scalar dy = a.position.y - other_pos.y;
            const Scalar dist_sq = dx * dx + dy * dy;
            const Scalar min_dist = a.radius + other_radius;
            if (!(dist_sq < min_dist * min_dist)) continue;
            Scalar nx = 0.0;
            Scalar ny = 0.0;
            Scalar dist = core::math::strictfp::sqrt(dist_sq);
            if (dist > 1e-9) {
                nx = dx / dist;
                ny = dy / dist;
            } else {  // same spot: split along x, decided by entity index
                dist = 0.0;
                nx = a.entity.index() < other.index() ? -1.0 : 1.0;
            }
            const Scalar overlap = min_dist - dist;
            const Scalar share = weight_a / (weight_a + weight_b);
            const Scalar amount = overlap * params.push_fraction * share;
            total.x += nx * amount;
            total.y += ny * amount;
        }
        const Scalar len_sq = total.x * total.x + total.y * total.y;
        if (len_sq > 0.0) {
            const Scalar len = core::math::strictfp::sqrt(len_sq);
            if (len > params.max_push_per_tick) {
                const Scalar k = params.max_push_per_tick / len;
                total.x *= k;
                total.y *= k;
            }
            any_push = true;
        }
        push[i] = total;
    }

    // Pass 2: apply (pool order), refusing pushes into blocked cells.
    bool moved = false;
    if (any_push) {
        for (std::size_t i = 0; i < agents.size(); ++i) {
            const Vec3& d = push[i];
            if (d.x == 0.0 && d.y == 0.0) continue;
            const Agent& a = agents[i];
            Vec3 target = a.position;
            const Vec3 both{a.position.x + d.x, a.position.y + d.y, a.position.z};
            const Vec3 only_x{a.position.x + d.x, a.position.y, a.position.z};
            const Vec3 only_y{a.position.x, a.position.y + d.y, a.position.z};
            if (!blocked_at(map, a.radius, both)) target = both;
            else if (d.x != 0.0 && !blocked_at(map, a.radius, only_x)) target = only_x;
            else if (d.y != 0.0 && !blocked_at(map, a.radius, only_y)) target = only_y;
            if (target.x != a.position.x || target.y != a.position.y) {
                world.get<Position>(a.entity).value = target;
                moved = true;
            }
        }
    }

    // Pass 3: stall detection for ordered agents; tidy up progress of unordered ones.
    for (const Agent& a : agents) {
        const Entity e = a.entity;
        if (!world.has<Destination>(e)) {
            if (world.has<MoveProgress>(e)) world.remove<MoveProgress>(e);
            continue;
        }
        const Destination dest = world.get<Destination>(e);
        const Vec3 now = world.get<Position>(e).value;
        if (!world.has<MoveProgress>(e)) {
            world.emplace<MoveProgress>(e, MoveProgress{now, 0, 0});
            continue;
        }
        MoveProgress progress = world.get<MoveProgress>(e);
        ++progress.ticks_in_window;
        if (progress.ticks_in_window >= MoveProgress::kWindowTicks) {
            const Vec3 moved_by = now - progress.anchor;
            const Scalar actual = core::math::strictfp::sqrt(moved_by.x * moved_by.x + moved_by.y * moved_by.y);
            const Scalar expected = dest.speed * dt_seconds * static_cast<Scalar>(MoveProgress::kWindowTicks);
            const bool stalled = actual < expected * params.min_progress_fraction;
            progress.stalled_windows = stalled ? progress.stalled_windows + 1 : 0;
            progress.anchor = now;
            progress.ticks_in_window = 0;

            const Vec3 to_goal = dest.target - now;
            const Scalar goal_dist = core::math::strictfp::sqrt(to_goal.x * to_goal.x + to_goal.y * to_goal.y);
            Scalar near_limit = params.near_goal_distance;
            if (a.radius * 4.0 > near_limit) near_limit = a.radius * 4.0;
            const std::uint32_t limit = goal_dist <= near_limit ? params.stalled_windows_near : params.stalled_windows_far;
            if (progress.stalled_windows >= limit) {  // give up: rest here
                if (world.has<Path>(e)) world.remove<Path>(e);
                world.remove<Destination>(e);
                world.remove<MoveProgress>(e);
                world.emplace<Velocity>(e, Velocity{Vec3{0.0, 0.0, 0.0}});
                continue;
            }
        }
        world.get<MoveProgress>(e) = progress;
    }
    return moved;
}

}  // namespace thalassa::sim::systems
