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

        // Entity is already close enough to the target.
        if (dist <= dest.arrival_radius) {
            pos.value = dest.target;

            world.emplace<Velocity>(e, Velocity{ Vec3{0.0, 0.0, 0.0} });
            world.remove<Destination>(e);
            return;
        }

        const Vec3 direction{
            delta.x / dist,
            delta.y / dist,
            delta.z / dist
        };

        // Distance the entity can travel during this simulation step.
        const core::Scalar step = dest.speed * dt_seconds;

        // Prevent overshooting the target.
        if (step >= dist) {
            pos.value = dest.target;
            world.emplace<Velocity>(e,Velocity{ Vec3{0.0, 0.0, 0.0} });
            world.remove<Destination>(e);
            return;
        }

        world.emplace<Velocity>(e, Velocity{ direction * dest.speed });
    });
}

void integration_system(World& world, core::Scalar dt_seconds) {
    world.each<Position, Velocity>([dt_seconds](Entity /*e*/, Position& pos, Velocity& vel) {
        pos.value += vel.value * dt_seconds;
    });
}

}  // namespace thalassa::sim::systems
