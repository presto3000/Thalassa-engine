#include <bit>
#include <cstddef>
#include <cstdint>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "thalassa/sim/components.hpp"
#include "thalassa/sim/sim_world.hpp"

using Catch::Approx;
using thalassa::core::ecs::Entity;
using thalassa::sim::SimWorld;
using thalassa::sim::components::Destination;
using thalassa::sim::components::Position;
using thalassa::sim::components::Velocity;

namespace {

// Runs `ticks` sim ticks and returns final Position.
Position simulate_single_mover(const thalassa::core::math::Vec3& start,
                                const thalassa::core::math::Vec3& target,
                                thalassa::core::Scalar speed,
                                int ticks) {
    SimWorld world(1);
    const Entity e = world.world().create_entity();
    world.world().emplace<Position>(e, Position{start});
    world.world().emplace<Destination>(e, Destination{target, speed, 0.05});

    for (int i = 0; i < ticks; ++i) {
        world.tick(thalassa::core::SimTick{static_cast<std::uint64_t>(i)});
    }
    return world.world().get<Position>(e);
}

}  // namespace

TEST_CASE("A single unit moves toward its destination each tick", "[movement]") {
    const Position after_one_tick =
        simulate_single_mover({0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}, 60.0, 1);

    // speed 60 units/sec at 1/60s tick = 1 unit per tick, straight along +X.
    REQUIRE(after_one_tick.value.x == Approx(1.0).epsilon(1e-9));
    REQUIRE(after_one_tick.value.y == Approx(0.0).margin(1e-9));
}

TEST_CASE("A unit arrives at its destination and stops (Destination component removed)", "[movement]") {
    SimWorld world(1);
    const Entity e = world.world().create_entity();
    world.world().emplace<Position>(e, Position{{0.0, 0.0, 0.0}});
    world.world().emplace<Destination>(e, Destination{{1.0, 0.0, 0.0}, 60.0, 0.1});

    // At 60 units/sec, 1 unit/tick: should arrive within ~1-2 ticks.
    for (int i = 0; i < 5; ++i) {
        world.tick(thalassa::core::SimTick{static_cast<std::uint64_t>(i)});
    }

    REQUIRE_FALSE(world.world().has<Destination>(e));
    const auto& pos = world.world().get<Position>(e);
    const thalassa::core::Scalar dist_to_target =
        (pos.value - thalassa::core::math::Vec3{1.0, 0.0, 0.0}).length();
    REQUIRE(dist_to_target <= 0.1);

    // Velocity should be zeroed once arrived.
    REQUIRE(world.world().get<Velocity>(e).value.x == 0.0);
    REQUIRE(world.world().get<Velocity>(e).value.y == 0.0);
}

TEST_CASE("An entity with Velocity but no Destination still integrates (reusable for knockback/projectiles)",
          "[movement]") {
    SimWorld world(1);
    const Entity e = world.world().create_entity();
    world.world().emplace<Position>(e, Position{{0.0, 0.0, 0.0}});
    world.world().emplace<Velocity>(e, Velocity{{10.0, 0.0, 0.0}});
    // Deliberately no Destination.

    world.tick(thalassa::core::SimTick{0});

    const auto& pos = world.world().get<Position>(e);
    REQUIRE(pos.value.x == Approx(10.0 / 60.0).epsilon(1e-9));
}

TEST_CASE("Determinism: 5000 units with randomized destinations produce identical checksums across two runs",
          "[movement][determinism]") {
    constexpr std::uint64_t kSeed = 2024;
    constexpr int kEntityCount = 5000;
    constexpr int kTicks = 400;  // enough for most units to travel typical distances and arrive

    auto run = []() {
        SimWorld world(kSeed);
        for (int i = 0; i < kEntityCount; ++i) {
            const Entity e = world.world().create_entity();
            const thalassa::core::Scalar sx = world.rng().next_scalar01() * 200.0 - 100.0;
            const thalassa::core::Scalar sy = world.rng().next_scalar01() * 200.0 - 100.0;
            const thalassa::core::Scalar tx = world.rng().next_scalar01() * 200.0 - 100.0;
            const thalassa::core::Scalar ty = world.rng().next_scalar01() * 200.0 - 100.0;
            const thalassa::core::Scalar speed = 10.0 + world.rng().next_scalar01() * 20.0;

            world.world().emplace<Position>(e, Position{{sx, sy, 0.0}});
            world.world().emplace<Destination>(e, Destination{{tx, ty, 0.0}, speed, 0.1});
        }

        for (int t = 0; t < kTicks; ++t) {
            world.tick(thalassa::core::SimTick{static_cast<std::uint64_t>(t)});
        }

        std::uint64_t checksum = 0;
        world.world().each<Position>([&checksum](Entity e, const Position& p) {
            auto mix = [&checksum](std::uint64_t v) {
                checksum ^= v;
                checksum *= 1099511628211ULL;
            };
            mix(e.index());
            mix(std::bit_cast<std::uint64_t>(p.value.x));
            mix(std::bit_cast<std::uint64_t>(p.value.y));
        });
        return checksum;
    };

    const std::uint64_t checksum_a = run();
    const std::uint64_t checksum_b = run();
    REQUIRE(checksum_a == checksum_b);
}

TEST_CASE("5000 units: most arrive at their destination within a generous tick budget", "[movement][determinism]") {
    // Correctness sanity check alongside the determinism check above: with
    // max travel distance ~283 units (diagonal of a 200x200 box) and
    // minimum speed 10 units/sec at 60 ticks/sec, worst case is ~1700
    // ticks. We run fewer ticks than that on purpose and just check a
    // reasonable majority have arrived, to keep the test fast; this is a
    // sanity check, not a strict correctness proof.
    SimWorld world(777);
    constexpr int kEntityCount = 5000;
    constexpr int kTicks = 1800;

    for (int i = 0; i < kEntityCount; ++i) {
        const Entity e = world.world().create_entity();
        const thalassa::core::Scalar sx = world.rng().next_scalar01() * 200.0 - 100.0;
        const thalassa::core::Scalar sy = world.rng().next_scalar01() * 200.0 - 100.0;
        const thalassa::core::Scalar tx = world.rng().next_scalar01() * 200.0 - 100.0;
        const thalassa::core::Scalar ty = world.rng().next_scalar01() * 200.0 - 100.0;
        const thalassa::core::Scalar speed = 10.0 + world.rng().next_scalar01() * 20.0;

        world.world().emplace<Position>(e, Position{{sx, sy, 0.0}});
        world.world().emplace<Destination>(e, Destination{{tx, ty, 0.0}, speed, 0.1});
    }

    for (int t = 0; t < kTicks; ++t) {
        world.tick(thalassa::core::SimTick{static_cast<std::uint64_t>(t)});
    }

    const std::size_t still_traveling = world.world().component_count<Destination>();
    // Allow a small tail of slow/far units to still be in transit.
    REQUIRE(still_traveling < kEntityCount / 20);  // < 5% still traveling
}
