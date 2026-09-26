#include <bit>
#include <cstdint>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "thalassa/sim/components.hpp"
#include "thalassa/sim/sim_world.hpp"

using Catch::Approx;
using thalassa::core::ecs::Entity;
using thalassa::core::math::Vec3;
using thalassa::sim::SimWorld;
using thalassa::sim::components::CombatStats;
using thalassa::sim::components::Health;
using thalassa::sim::components::Position;
using thalassa::sim::components::Projectile;
using thalassa::sim::components::Team;

namespace {

Entity spawn_unit(SimWorld& world, Vec3 position, std::int32_t team, thalassa::core::Scalar max_health,
                   thalassa::core::Scalar attack_damage, thalassa::core::Scalar attack_range,
                   thalassa::core::Scalar attack_cooldown) {
    const Entity e = world.world().create_entity();
    world.world().emplace<Position>(e, Position{position});
    world.world().emplace<Team>(e, Team{team});
    world.world().emplace<Health>(e, Health{max_health, max_health});
    world.world().emplace<CombatStats>(e, CombatStats{attack_damage, attack_range, attack_cooldown, 0.0});
    return e;
}

void run_ticks(SimWorld& world, int count) {
    for (int i = 0; i < count; ++i) {
        world.tick(thalassa::core::SimTick{static_cast<std::uint64_t>(i)});
    }
}

}  // namespace

TEST_CASE("A unit in range fires a projectile that damages its target", "[combat]") {
    SimWorld world(1);
    const Entity attacker = spawn_unit(world, {0.0, 0.0, 0.0}, /*team=*/0, 100.0, /*dmg=*/25.0, /*range=*/20.0, 1.0);
    const Entity target = spawn_unit(world, {10.0, 0.0, 0.0}, /*team=*/1, 100.0, 0.0, 0.0, 1.0);

    // Projectile speed is 40 units/sec; distance 10 units => ~0.25s => ~15
    // ticks at 60Hz to travel + a tick to fire. Run generously long enough.
    run_ticks(world, 60);

    REQUIRE(world.world().is_alive(attacker));
    REQUIRE(world.world().is_alive(target));
    REQUIRE(world.world().get<Health>(target).current == Approx(75.0).epsilon(1e-6));
    // Cooldown 1s = 60 ticks: exactly one shot should have landed by tick 60.
}

TEST_CASE("A unit out of range never fires", "[combat]") {
    SimWorld world(1);
    spawn_unit(world, {0.0, 0.0, 0.0}, 0, 100.0, 25.0, /*range=*/5.0, 1.0);
    const Entity target = spawn_unit(world, {100.0, 0.0, 0.0}, 1, 100.0, 0.0, 0.0, 1.0);

    run_ticks(world, 120);

    REQUIRE(world.world().get<Health>(target).current == 100.0);
    REQUIRE(world.world().component_count<Projectile>() == 0);
}

TEST_CASE("Same-team units never target each other", "[combat]") {
    SimWorld world(1);
    spawn_unit(world, {0.0, 0.0, 0.0}, 0, 100.0, 25.0, 20.0, 1.0);
    const Entity ally = spawn_unit(world, {5.0, 0.0, 0.0}, 0, 100.0, 25.0, 20.0, 1.0);

    run_ticks(world, 60);

    REQUIRE(world.world().get<Health>(ally).current == 100.0);
}

TEST_CASE("A unit respects its cooldown and does not fire on every tick", "[combat]") {
    SimWorld world(1);
    // Very high cooldown so it fires at most once in the test window.
    spawn_unit(world, {0.0, 0.0, 0.0}, 0, 100.0, 10.0, /*range=*/1000.0, /*cooldown=*/10.0);
    const Entity target = spawn_unit(world, {5.0, 0.0, 0.0}, 1, 1000.0, 0.0, 0.0, 1.0);

    run_ticks(world, 600);  // 10 simulated seconds: enough for exactly one shot at a 10s cooldown

    const auto damage_taken = 1000.0 - world.world().get<Health>(target).current;
    REQUIRE(damage_taken == Approx(10.0).epsilon(1e-6));
}

TEST_CASE("A unit reduced to zero health is removed from the world", "[combat]") {
    SimWorld world(1);
    spawn_unit(world, {0.0, 0.0, 0.0}, 0, 100.0, /*dmg=*/50.0, /*range=*/20.0, /*cooldown=*/0.1);
    const Entity target = spawn_unit(world, {5.0, 0.0, 0.0}, 1, 50.0, 0.0, 0.0, 1.0);

    run_ticks(world, 60);  // one 50-damage hit is enough to kill a 50-HP target

    REQUIRE_FALSE(world.world().is_alive(target));
    REQUIRE(world.world().component_count<Health>() == 1);  // only the attacker remains
}

TEST_CASE("Combat resolution is deterministic across two independent identical runs", "[combat][determinism]") {
    auto run = []() {
        SimWorld world(2024);
        // Randomized (not mirror-symmetric) starting positions: a
        // perfectly symmetric grid makes every unit's fight literally
        // identical to its mirror-image opponent's, which is real
        // determinism but doesn't exercise target-selection/tie-breaking
        // the way an asymmetric fight does
        for (int i = 0; i < 20; ++i) {
            const auto x = world.rng().next_scalar01() * 40.0;
            spawn_unit(world, {x, 0.0, 0.0}, 0, 100.0, 12.0, 15.0, 0.5);
        }
        for (int i = 0; i < 20; ++i) {
            const auto x = world.rng().next_scalar01() * 40.0;
            spawn_unit(world, {x, 10.0, 0.0}, 1, 100.0, 12.0, 15.0, 0.5);
        }
        // 150 ticks (2.5s) leaves a genuine mid-fight state — casualties
        // and a spread of health values among survivors — rather than
        // running long enough for the checksum to trivially settle at "0"
        // once every unit is dead.
        run_ticks(world, 150);

        std::uint64_t checksum = 0;
        world.world().each<Health>([&checksum](Entity e, const Health& h) {
            auto mix = [&checksum](std::uint64_t v) {
                checksum ^= v;
                checksum *= 1099511628211ULL;
            };
            mix(e.index());
            mix(std::bit_cast<std::uint64_t>(h.current));
        });
        return checksum;
    };

    const std::uint64_t checksum_a = run();
    const std::uint64_t checksum_b = run();
    REQUIRE(checksum_a == checksum_b);
    REQUIRE(checksum_a != 0);  // sanity: this scenario must produce a non-trivial (not "everyone's dead") state
}
