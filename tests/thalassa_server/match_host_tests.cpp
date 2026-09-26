#include <bit>
#include <cstdint>

#include <catch2/catch_test_macros.hpp>

#include "thalassa/server/match_host.hpp"
#include "thalassa/sim/commands.hpp"
#include "thalassa/sim/components.hpp"

using thalassa::core::MatchId;
using thalassa::core::SimTick;
using thalassa::core::ecs::Entity;
using thalassa::server::MatchHost;
using thalassa::sim::Command;
using thalassa::sim::SimWorld;
using thalassa::sim::components::Position;

namespace {

std::uint64_t checksum_of(const SimWorld& world) {
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
}

}  // namespace

TEST_CASE("MatchHost can create and independently tick multiple matches", "[match_host]") {
    MatchHost host;
    const MatchId a = host.create_match(1);
    const MatchId b = host.create_match(2);

    REQUIRE(host.has_match(a));
    REQUIRE(host.has_match(b));
    REQUIRE(host.match_count() == 2);
    REQUIRE(a != b);

    host.submit_command(a, SimTick{0}, Command::spawn_unit({0.0, 0.0, 0.0}, {}, 0, 100.0, 10.0, 5.0, 1.0));

    host.tick_all();

    REQUIRE(thalassa::core::to_u64(host.match_tick(a)) == 1);
    REQUIRE(thalassa::core::to_u64(host.match_tick(b)) == 1);
    REQUIRE(host.match_world(a).world().alive_count() == 1);
    REQUIRE(host.match_world(b).world().alive_count() == 0);  // command to A never affects B
}

TEST_CASE("destroy_match removes a match and its id is never reused", "[match_host]") {
    MatchHost host;
    const MatchId a = host.create_match(1);
    host.destroy_match(a);

    REQUIRE_FALSE(host.has_match(a));

    const MatchId b = host.create_match(1);
    REQUIRE(b != a);
}

TEST_CASE("Two matches with different commands evolve completely independently", "[match_host]") {
    MatchHost host;
    const MatchId a = host.create_match(42);
    const MatchId b = host.create_match(42);  // same seed, different commands

    host.submit_command(a, SimTick{0}, Command::spawn_unit({0.0, 0.0, 0.0}, {}, 0, 100.0, 10.0, 15.0, 1.0));
    host.submit_command(b, SimTick{0}, Command::spawn_unit({0.0, 0.0, 0.0}, {}, 0, 100.0, 10.0, 15.0, 1.0));
    host.submit_command(b, SimTick{0}, Command::spawn_unit({5.0, 0.0, 0.0}, {}, 1, 100.0, 10.0, 15.0, 1.0));

    for (int i = 0; i < 20; ++i) {
        host.tick_all();
    }

    REQUIRE(host.match_world(a).world().alive_count() == 1);  // only ever had one unit
    REQUIRE(host.match_world(b).world().alive_count() <= 2);  // may have taken casualties from combat
    REQUIRE(checksum_of(host.match_world(a)) != checksum_of(host.match_world(b)));
}

TEST_CASE("MatchHost.tick_all() produces the same result as driving an equivalent SimWorld directly",
          "[match_host][determinism]") {
    constexpr std::uint64_t kSeed = 7;

    MatchHost host;
    const MatchId m = host.create_match(kSeed);
    const auto spawn_a = Command::spawn_unit({0.0, 0.0, 0.0}, {}, 0, 100.0, 12.0, 15.0, 0.5);
    const auto spawn_b = Command::spawn_unit({8.0, 0.0, 0.0}, {}, 1, 100.0, 12.0, 15.0, 0.5);
    host.submit_command(m, SimTick{0}, spawn_a);
    host.submit_command(m, SimTick{0}, spawn_b);

    for (int i = 0; i < 60; ++i) {
        host.tick_all();
    }

    SimWorld reference(kSeed);
    reference.tick(SimTick{0}, {spawn_a, spawn_b});
    for (int i = 1; i < 60; ++i) {
        reference.tick(SimTick{static_cast<std::uint64_t>(i)});
    }

    REQUIRE(checksum_of(host.match_world(m)) == checksum_of(reference));
}

TEST_CASE("submit_command targeting an already-past tick is rescheduled, not dropped", "[match_host]") {
    MatchHost host;
    const MatchId m = host.create_match(1);

    host.tick_all();  // current tick is now 1
    host.tick_all();  // current tick is now 2

    // Target tick 0 is already in the past; the command must still take
    // effect (on the next tick that hasn't run yet) rather than vanish.
    host.submit_command(m, SimTick{0}, Command::spawn_unit({1.0, 1.0, 0.0}, {}, 0, 100.0, 10.0, 5.0, 1.0));
    host.tick_all();

    REQUIRE(host.match_world(m).world().alive_count() == 1);
}
