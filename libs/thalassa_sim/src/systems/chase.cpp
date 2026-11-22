#include "thalassa/sim/systems/chase.hpp"

#include <vector>

#include "thalassa/core/math/vector.hpp"
#include "thalassa/sim/components.hpp"
#include "thalassa/sim/nav/path_planner.hpp"

namespace thalassa::sim::systems {

using core::ecs::Entity;
using core::math::Vec3;
using components::Chase;
using components::CombatStats;
using components::Destination;
using components::Health;
using components::Path;
using components::Position;
using components::Velocity;

namespace {
void stop_walking(core::ecs::World& world, Entity e) {
    if (world.has<Path>(e)) world.remove<Path>(e);
    if (world.has<Destination>(e)) {
        world.remove<Destination>(e);
        world.emplace<Velocity>(e, Velocity{Vec3{0.0, 0.0, 0.0}});
    }
}
}  // namespace

void chase_system(core::ecs::World& world, nav::PathPlanner* planner) {
    std::vector<Entity> chasers;
    world.each<Chase>([&](Entity e, Chase&) { chasers.push_back(e); });

    for (const Entity e : chasers) {
        if (!world.has<Chase>(e)) continue;
        Chase chase = world.get<Chase>(e);

        const bool attacker_ok = world.has<Position>(e) && world.has<CombatStats>(e);
        const Entity target = chase.target;
        const bool target_ok = world.is_alive(target) && world.has<Position>(target) &&
                               (!world.has<Health>(target) || world.get<Health>(target).current > 0.0);
        if (!attacker_ok || !target_ok) {
            world.remove<Chase>(e);
            stop_walking(world, e);
            continue;
        }

        const Vec3 me = world.get<Position>(e).value;
        const Vec3 them = world.get<Position>(target).value;
        const core::Scalar reach = world.get<CombatStats>(e).attack_range * kChaseReachFraction;
        const core::Scalar dist = (them - me).length();

        if (dist <= reach) {
            stop_walking(world, e);
            chase.ticks_since_plan = Chase::kMinReplanTicks;  // re-engage promptly if it walks away
            world.get<Chase>(e) = chase;
            continue;
        }

        if (chase.ticks_since_plan < Chase::kMinReplanTicks) ++chase.ticks_since_plan;
        bool replan = false;
        if (chase.ticks_since_plan >= Chase::kMinReplanTicks) {
            if (!world.has<Destination>(e)) {
                replan = true;
            } else {
                const core::Scalar goal_error = (world.get<Destination>(e).target - them).length();
                replan = goal_error > kChaseReplanDistance;
            }
        }
        if (replan) {
            if (world.has<Path>(e)) world.remove<Path>(e);
            world.emplace<Destination>(e, Destination{them, chase.speed, 0.5});
            if (planner != nullptr) planner->plan(world, e);
            chase.ticks_since_plan = 0;
        }
        world.get<Chase>(e) = chase;
    }
}

}  // namespace thalassa::sim::systems
