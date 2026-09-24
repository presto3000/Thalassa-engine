#include <bit>
#include <cstdint>

#include <catch2/catch_test_macros.hpp>

#include "thalassa/sim/components.hpp"
#include "thalassa/sim/replay.hpp"
#include "thalassa/sim/sim_world.hpp"

using thalassa::core::ecs::Entity;
using thalassa::core::serialization::ByteReader;
using thalassa::core::serialization::ByteWriter;
using thalassa::sim::SimWorld;
using thalassa::sim::components::Destination;
using thalassa::sim::components::Position;

namespace {

void populate(SimWorld& world, std::uint32_t entity_count) {
    for (std::uint32_t i = 0; i < entity_count; ++i) {
        const Entity e = world.world().create_entity();
        const auto sx = world.rng().next_scalar01() * 200.0 - 100.0;
        const auto sy = world.rng().next_scalar01() * 200.0 - 100.0;
        const auto tx = world.rng().next_scalar01() * 200.0 - 100.0;
        const auto ty = world.rng().next_scalar01() * 200.0 - 100.0;
        const auto speed = 10.0 + world.rng().next_scalar01() * 20.0;
        world.world().emplace<Position>(e, Position{{sx, sy, 0.0}});
        world.world().emplace<Destination>(e, Destination{{tx, ty, 0.0}, speed, 0.1});
    }
}

std::uint64_t checksum_of(const SimWorld& world) {
    std::uint64_t result = 0;
    // world() is const here, but each<T> is only defined non-const on World today;
    // cast away const for this test-only checksum helper rather than adding an
    // unused const overload to the public API speculatively.
    auto& mutable_world = const_cast<thalassa::core::ecs::World&>(world.world());
    mutable_world.each<Position>([&result](Entity e, const Position& p) {
        auto mix = [&result](std::uint64_t v) {
            result ^= v;
            result *= 1099511628211ULL;
        };
        mix(e.index());
        mix(std::bit_cast<std::uint64_t>(p.value.x));
        mix(std::bit_cast<std::uint64_t>(p.value.y));
    });
    return result;
}

}  // namespace

TEST_CASE("save_world/load_world round-trips an empty world", "[replay]") {
    SimWorld original(1);
    ByteWriter writer;
    thalassa::sim::save_world(original, writer);

    SimWorld restored(999);  // different seed on purpose: load must override it
    ByteReader reader(writer.bytes());
    thalassa::sim::load_world(restored, reader);

    REQUIRE(restored.ticks_simulated() == 0);
    REQUIRE(restored.world().alive_count() == 0);
}

TEST_CASE("save_world/load_world round-trips entities, components, RNG, and tick count", "[replay]") {
    SimWorld original(42);
    populate(original, 500);
    for (int t = 0; t < 50; ++t) {
        original.tick(thalassa::core::SimTick{static_cast<std::uint64_t>(t)});
    }

    ByteWriter writer;
    thalassa::sim::save_world(original, writer);

    SimWorld restored(1);  // different seed: must be irrelevant after load
    ByteReader reader(writer.bytes());
    thalassa::sim::load_world(restored, reader);

    REQUIRE(restored.ticks_simulated() == original.ticks_simulated());
    REQUIRE(restored.world().alive_count() == original.world().alive_count());
    REQUIRE(restored.world().component_count<Position>() == original.world().component_count<Position>());
    REQUIRE(restored.world().component_count<Destination>() == original.world().component_count<Destination>());
    REQUIRE(checksum_of(restored) == checksum_of(original));

    // RNG continuation must match too: draw the same number of values from
    // both and compare, proving state (not just seed) was restored.
    for (int i = 0; i < 10; ++i) {
        REQUIRE(restored.rng().next_u32() == original.rng().next_u32());
    }
}

TEST_CASE("A world resumed from a snapshot simulates identically to one that was never interrupted",
          "[replay][determinism]") {
    constexpr std::uint64_t kSeed = 2024;
    constexpr std::uint32_t kEntityCount = 2000;
    constexpr int kTotalTicks = 200;
    constexpr int kSaveAtTick = 80;

    // Baseline: uninterrupted run for the full duration.
    SimWorld baseline(kSeed);
    populate(baseline, kEntityCount);
    for (int t = 0; t < kTotalTicks; ++t) {
        baseline.tick(thalassa::core::SimTick{static_cast<std::uint64_t>(t)});
    }

    // Interrupted: run halfway, save, load into a *fresh* world, continue.
    SimWorld first_half(kSeed);
    populate(first_half, kEntityCount);
    for (int t = 0; t < kSaveAtTick; ++t) {
        first_half.tick(thalassa::core::SimTick{static_cast<std::uint64_t>(t)});
    }

    ByteWriter writer;
    thalassa::sim::save_world(first_half, writer);

    SimWorld second_half(0xFFFFFFFF);  // seed is irrelevant, load overrides it
    ByteReader reader(writer.bytes());
    thalassa::sim::load_world(second_half, reader);

    for (int t = kSaveAtTick; t < kTotalTicks; ++t) {
        second_half.tick(thalassa::core::SimTick{static_cast<std::uint64_t>(t)});
    }

    REQUIRE(second_half.ticks_simulated() == baseline.ticks_simulated());
    REQUIRE(checksum_of(second_half) == checksum_of(baseline));
}
