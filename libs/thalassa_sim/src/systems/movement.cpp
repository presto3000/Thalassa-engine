#include "thalassa/sim/systems/movement.hpp"

#include "thalassa/core/math/vector.hpp"
#include "thalassa/sim/components.hpp"

namespace thalassa::sim::systems {

using core::math::Vec3;
using core::ecs::Entity;
using core::ecs::World;
using components::Destination;
using components::Position;
using components::Velocity;

void steering_system(World& world, core::Scalar dt_seconds) {
    world.each<Position, Destination>([&world, dt_seconds](Entity e, Position& pos, Destination& dest) {
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
