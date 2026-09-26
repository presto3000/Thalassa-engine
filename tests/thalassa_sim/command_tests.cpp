#include <bit>
#include <cstdint>

#include <catch2/catch_test_macros.hpp>

#include "thalassa/sim/commands.hpp"
#include "thalassa/sim/components.hpp"
#include "thalassa/sim/sim_world.hpp"

using thalassa::core::ecs::Entity;
using thalassa::core::math::Vec3;
using thalassa::sim::Command;
using thalassa::sim::CommandLog;
using thalassa::sim::SimWorld;
using thalassa::sim::apply_commands;
using thalassa::sim::components::CombatStats;
using thalassa::sim::components::Destination;
using thalassa::sim::components::Health;
using thalassa::sim::components::Owner;
using thalassa::sim::components::Position;
using thalassa::sim::components::Team;

TEST_CASE("apply_commands: SpawnUnit creates a fully-formed unit", "[commands]") {
    SimWorld world(1);
    const std::vector<Command> commands{
        Command::spawn_unit({1.0, 2.0, 0.0}, thalassa::core::PlayerId{3}, 7, 80.0, 15.0, 12.0, 0.75),
    };

    const std::vector<Entity> spawned = apply_commands(world.world(), commands);

    REQUIRE(spawned.size() == 1);
    const Entity e = spawned[0];
    REQUIRE(world.world().is_alive(e));
    REQUIRE(world.world().get<Position>(e).value == Vec3{1.0, 2.0, 0.0});
    REQUIRE(world.world().get<Owner>(e).player == thalassa::core::PlayerId{3});
    REQUIRE(world.world().get<Team>(e).id == 7);
    REQUIRE(world.world().get<Health>(e).current == 80.0);
    REQUIRE(world.world().get<Health>(e).max == 80.0);
    REQUIRE(world.world().get<CombatStats>(e).attack_damage == 15.0);
    REQUIRE(world.world().get<CombatStats>(e).attack_range == 12.0);
    REQUIRE(world.world().get<CombatStats>(e).attack_cooldown_seconds == 0.75);
}

TEST_CASE("apply_commands: MoveTo gives an existing entity a Destination", "[commands]") {
    SimWorld world(1);
    const std::vector<Entity> spawned =
        apply_commands(world.world(), {Command::spawn_unit({0.0, 0.0, 0.0}, {}, 0, 100.0, 10.0, 5.0, 1.0)});
    const Entity e = spawned[0];

    (void)apply_commands(world.world(), {Command::move_to(e, {50.0, 0.0, 0.0}, 20.0)});

    REQUIRE(world.world().has<Destination>(e));
    REQUIRE(world.world().get<Destination>(e).target == Vec3{50.0, 0.0, 0.0});
    REQUIRE(world.world().get<Destination>(e).speed == 20.0);
}

TEST_CASE("apply_commands: MoveTo for a dead/nonexistent entity is silently ignored", "[commands]") {
    SimWorld world(1);
    const Entity nonexistent{999, 0};

    // Must not assert/abort.
    REQUIRE_NOTHROW([&] { (void)apply_commands(world.world(), {Command::move_to(nonexistent, {1.0, 1.0, 0.0}, 5.0)}); }());
}

TEST_CASE("apply_commands returns spawned entities in spawn-command order, skipping non-spawn commands",
          "[commands]") {
    SimWorld world(1);
    const Entity dummy = world.world().create_entity();

    const std::vector<Command> commands{
        Command::spawn_unit({0.0, 0.0, 0.0}, {}, 0, 100.0, 10.0, 5.0, 1.0),
        Command::move_to(dummy, {1.0, 0.0, 0.0}, 5.0),
        Command::spawn_unit({1.0, 0.0, 0.0}, {}, 1, 100.0, 10.0, 5.0, 1.0),
    };

    const std::vector<Entity> spawned = apply_commands(world.world(), commands);
    REQUIRE(spawned.size() == 2);
    REQUIRE(world.world().get<Team>(spawned[0]).id == 0);
    REQUIRE(world.world().get<Team>(spawned[1]).id == 1);
}

namespace {

// Builds a small two-team skirmish as a CommandLog: spawn both sides at
// tick 0, then issue a move-toward-the-enemy-line order for each unit a
// few ticks later (an "attack-move", the simplest input that reliably
// brings both sides into weapon range without needing real player/AI
// decision-making for this test).
CommandLog build_skirmish_log(std::vector<Command>& tick0_spawn_commands_out) {
    CommandLog log;

    constexpr int kUnitsPerTeam = 15;
    std::vector<Command> spawn_commands;
    for (int i = 0; i < kUnitsPerTeam; ++i) {
        spawn_commands.push_back(Command::spawn_unit({static_cast<double>(i) * 2.0, 0.0, 0.0}, {}, 0, 100.0, 12.0,
                                                       15.0, 0.5));
    }
    for (int i = 0; i < kUnitsPerTeam; ++i) {
        spawn_commands.push_back(
            Command::spawn_unit({static_cast<double>(i) * 2.0, 30.0, 0.0}, {}, 1, 100.0, 12.0, 15.0, 0.5));
    }
    for (const Command& c : spawn_commands) {
        log.add(0, c);
    }
    tick0_spawn_commands_out = spawn_commands;

    // Note: MoveTo commands referencing specific spawned entities are added
    // by the caller once it knows the resulting Entity ids (see
    // run_skirmish below) — CommandLog itself doesn't care when entries
    // are added, only what tick they're recorded against.
    return log;
}

// Runs `log` against a fresh SimWorld(seed) for `total_ticks`, applying
// whatever commands_at(tick) returns each tick, and returns a checksum of
// final Health+Position state. This is the actual "replay a fight from
// recorded commands" operation: nothing but the seed and the log feeds in.
std::uint64_t run_from_log(std::uint64_t seed, const CommandLog& log, int total_ticks) {
    SimWorld world(seed);
    for (int t = 0; t < total_ticks; ++t) {
        const auto* commands = log.commands_at(static_cast<std::uint64_t>(t));
        if (commands != nullptr) {
            world.tick(thalassa::core::SimTick{static_cast<std::uint64_t>(t)}, *commands);
        } else {
            world.tick(thalassa::core::SimTick{static_cast<std::uint64_t>(t)});
        }
    }

    std::uint64_t checksum = 0;
    world.world().each<Health>([&checksum](Entity e, const Health& h) {
        auto mix = [&checksum](std::uint64_t v) {
            checksum ^= v;
            checksum *= 1099511628211ULL;
        };
        mix(e.index());
        mix(std::bit_cast<std::uint64_t>(h.current));
    });
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

TEST_CASE("A full fight replayed from a recorded CommandLog reproduces identical results", "[commands][replay]") {
    constexpr std::uint64_t kSeed = 555;
    constexpr int kTotalTicks = 400;

    // Build the log once (spawns at tick 0, move orders at tick 30 once we
    // know the spawned entity ids — apply the spawn commands to a
    // throwaway World first purely to learn which Entity each spawn
    // produced, matching how a real client would learn entity ids back
    // from the server after issuing spawn commands).
    std::vector<Command> spawn_commands;
    CommandLog log = build_skirmish_log(spawn_commands);

    SimWorld id_discovery_world(kSeed);
    const std::vector<Entity> spawned = apply_commands(id_discovery_world.world(), spawn_commands);
    REQUIRE(spawned.size() == 30);

    for (std::size_t i = 0; i < 15; ++i) {
        log.add(30, Command::move_to(spawned[i], {static_cast<double>(i) * 2.0, 30.0, 0.0}, 8.0));
    }
    for (std::size_t i = 15; i < 30; ++i) {
        log.add(30, Command::move_to(spawned[i], {static_cast<double>(i - 15) * 2.0, 0.0, 0.0}, 8.0));
    }

    const std::uint64_t checksum_a = run_from_log(kSeed, log, kTotalTicks);
    const std::uint64_t checksum_b = run_from_log(kSeed, log, kTotalTicks);

    REQUIRE(checksum_a == checksum_b);
}

TEST_CASE("A replayed fight actually produces combat outcomes (not just a trivially-empty match)",
          "[commands][replay]") {
    constexpr std::uint64_t kSeed = 555;
    constexpr int kTotalTicks = 400;

    std::vector<Command> spawn_commands;
    CommandLog log = build_skirmish_log(spawn_commands);

    SimWorld id_discovery_world(kSeed);
    const std::vector<Entity> spawned = apply_commands(id_discovery_world.world(), spawn_commands);

    for (std::size_t i = 0; i < 15; ++i) {
        log.add(30, Command::move_to(spawned[i], {static_cast<double>(i) * 2.0, 30.0, 0.0}, 8.0));
    }
    for (std::size_t i = 15; i < 30; ++i) {
        log.add(30, Command::move_to(spawned[i], {static_cast<double>(i - 15) * 2.0, 0.0, 0.0}, 8.0));
    }

    SimWorld world(kSeed);
    for (int t = 0; t < kTotalTicks; ++t) {
        const auto* commands = log.commands_at(static_cast<std::uint64_t>(t));
        if (commands != nullptr) {
            world.tick(thalassa::core::SimTick{static_cast<std::uint64_t>(t)}, *commands);
        } else {
            world.tick(thalassa::core::SimTick{static_cast<std::uint64_t>(t)});
        }
    }

    // Some combat must have actually happened: either casualties, or
    // survivors below full health. Both sides starting at 30 total units
    // with weapons in range of each other for hundreds of ticks should
    // produce at least one of these.
    const std::size_t survivors = world.world().component_count<Health>();
    bool any_damage_taken = false;
    world.world().each<Health>([&any_damage_taken](Entity, const Health& h) {
        if (h.current < h.max) {
            any_damage_taken = true;
        }
    });

    REQUIRE((survivors < 30 || any_damage_taken));
}
