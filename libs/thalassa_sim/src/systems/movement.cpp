#include "thalassa/sim/systems/movement.hpp"

#include "thalassa/core/math/vector.hpp"
#include "thalassa/sim/components.hpp"
#include "thalassa/sim/nav/path_planner.hpp"

namespace thalassa::sim::systems {

using core::math::Vec3;
using core::ecs::Entity;
using core::ecs::World;
using components::Destination;
using components::Path;
using components::Position;
using components::Velocity;

namespace {

// One entity's movement along its Path for one tick. See steering_system's header comment.
void follow_path(World& world, Entity e, Position& pos, Destination& dest, core::Scalar dt_seconds,
                 nav::PathPlanner* planner) {
    Path path = world.get<Path>(e);  // local copy: the pool may change below (replanning)
    const core::Scalar step = dest.speed * dt_seconds;
    const Vec3 zero{0.0, 0.0, 0.0};
    if (!(step > 0.0)) {
        world.emplace<Velocity>(e, Velocity{zero});
        return;
    }

    core::Scalar budget = step;  // distance we may still travel this tick
    Vec3 at = pos.value;
    while (path.next < path.count) {
        const Vec3 delta = path.points[path.next] - at;
        const core::Scalar dist = delta.length();
        if (dist > budget) break;
        at = path.points[path.next];  // reached: land exactly on the waypoint
        budget -= dist;
        ++path.next;
        if (!(budget > 0.0)) break;
    }
    pos.value = at;

    if (path.next < path.count) {
        // Still walking: aim at the next waypoint, scaled so integration moves exactly the
        // remaining budget (the full speed unless a waypoint was consumed this tick).
        const Vec3 delta = path.points[path.next] - at;
        const core::Scalar dist = delta.length();
        if (!(dist > 0.0)) {  // budget ran out exactly on a waypoint: wait for next tick
            world.emplace<Velocity>(e, Velocity{zero});
            world.get<Path>(e) = path;
            return;
        }
        const Vec3 direction{delta.x / dist, delta.y / dist, delta.z / dist};
        const core::Scalar fraction = budget / step;  // exactly 1.0 when nothing was consumed
        world.emplace<Velocity>(e, Velocity{direction * (dest.speed * fraction)});
        world.get<Path>(e) = path;
        return;
    }

    // Route finished.
    world.emplace<Velocity>(e, Velocity{zero});
    if ((path.flags & Path::kTruncated) != 0 && planner != nullptr) {
        // More route exists beyond what fit in the Path: plan the next stretch from here.
        world.remove<Path>(e);
        planner->plan(world, e);
        return;
    }
    world.remove<Path>(e);
    world.remove<Destination>(e);
}

}  // namespace

void steering_system(World& world, core::Scalar dt_seconds) { steering_system(world, dt_seconds, nullptr); }

void steering_system(World& world, core::Scalar dt_seconds, nav::PathPlanner* planner) {
    world.each<Position, Destination>([&world, dt_seconds, planner](Entity e, Position& pos, Destination& dest) {
        if (world.has<Path>(e)) {
            follow_path(world, e, pos, dest, dt_seconds, planner);
            return;
        }

        const Vec3 delta = dest.target - pos.value;
        const core::Scalar dist = delta.length();
        const core::Scalar step = dest.speed * dt_seconds;

        // Arrived already, or this tick's travel would reach/pass the
        // target: snap exactly onto it rather than overshooting. See the
        // header comment for why this snap is required for correctness,
        // not just tidiness — without it, small arrival_radius values
        // relative to step size cause permanent oscillation around the
        // target and the entity never "arrives".
        if (dist <= dest.arrival_radius || dist <= step) {
            pos.value = dest.target;
            // Unconditional emplace (not has-then-get): an entity can
            // arrive on its very first tick, before ever having had a
            // Velocity component created for it by the "still moving"
            // branch below. Any entity that was ever steered should end
            // up with a well-defined (zero) Velocity once stopped, rather
            // than having no Velocity component at all — both because
            // presentation-layer code may reasonably assume "has
            // Destination at some point implies has Velocity", and
            // because emplace() is idempotent/overwriting either way, so
            // this is never wrong, just occasionally a no-op-shaped write.
            world.emplace<Velocity>(e, Velocity{Vec3{0.0, 0.0, 0.0}});
            world.remove<Destination>(e);
            return;
        }

        // Manual unit-direction (delta / dist) instead of delta.normalized()
        // to avoid computing length() twice (normalized() would take
        // another sqrt internally).
        const Vec3 direction{delta.x / dist, delta.y / dist, delta.z / dist};
        const Vec3 desired_velocity = direction * dest.speed;
        world.emplace<Velocity>(e, Velocity{desired_velocity});
    });
}

void integration_system(World& world, core::Scalar dt_seconds) {
    world.each<Position, Velocity>([dt_seconds](Entity /*e*/, Position& pos, Velocity& vel) {
        pos.value += vel.value * dt_seconds;
    });
}

}  // namespace thalassa::sim::systems
