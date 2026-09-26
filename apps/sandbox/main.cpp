#include <algorithm>
#include <bit>
#include <cstdint>
#include <cstdlib>
#include <limits>

#include "thalassa/core/core.hpp"
#include "thalassa/net/predicted_client.hpp"
#include "thalassa/server/headless_app.hpp"
#include "thalassa/server/match_host.hpp"
#include "thalassa/sim/commands.hpp"
#include "thalassa/sim/components.hpp"
#include "thalassa/sim/sim_world.hpp"

// Sandbox: a
// determinism smoke test that exercises thalassa_core's ECS/Rng/math and
// thalassa_sim's movement + combat systems together end-to-end.
// Fast-forward (non-real-time) so it's fast to run twice and diff. See
// tests/thalassa_sim/movement_tests.cpp and combat_tests.cpp/command_tests.cpp
// for the versions of this pinned down as automated tests

using thalassa::sim::components::Destination;
using thalassa::sim::components::Health;
using thalassa::sim::components::Position;

namespace {

struct RunResult {
    std::uint64_t checksum = 0;
    std::size_t still_traveling = 0;
    std::size_t spatial_hits_near_origin = 0;
};

RunResult run_movement_smoke_test(std::uint64_t seed, std::uint32_t entity_count, int ticks) {
    thalassa::sim::SimWorld world(seed);

    for (std::uint32_t i = 0; i < entity_count; ++i) {
        const auto e = world.world().create_entity();
        const thalassa::core::Scalar sx = world.rng().next_scalar01() * 200.0 - 100.0;
        const thalassa::core::Scalar sy = world.rng().next_scalar01() * 200.0 - 100.0;
        const thalassa::core::Scalar tx = world.rng().next_scalar01() * 200.0 - 100.0;
        const thalassa::core::Scalar ty = world.rng().next_scalar01() * 200.0 - 100.0;
        const thalassa::core::Scalar speed = 10.0 + world.rng().next_scalar01() * 20.0;

        world.world().emplace<Position>(e, Position{{sx, sy, 0.0}});
        world.world().emplace<Destination>(e, Destination{{tx, ty, 0.0}, speed, 0.1});
    }

    for (int t = 0; t < ticks; ++t) {
        world.tick(thalassa::core::SimTick{static_cast<std::uint64_t>(t)});
    }

    RunResult result;
    world.world().each<Position>([&result](thalassa::core::ecs::Entity e, const Position& p) {
        auto mix = [&result](std::uint64_t v) {
            result.checksum ^= v;
            result.checksum *= 1099511628211ULL;
        };
        mix(e.index());
        mix(std::bit_cast<std::uint64_t>(p.value.x));
        mix(std::bit_cast<std::uint64_t>(p.value.y));
    });
    result.still_traveling = world.world().component_count<Destination>();

    std::vector<thalassa::core::ecs::Entity> nearby;
    world.spatial_index().query_radius({0.0, 0.0, 0.0}, 20.0, nearby);
    result.spatial_hits_near_origin = nearby.size();

    return result;
}

struct BattleResult {
    std::uint64_t checksum = 0;
    int survivors = 0;
    int casualties = 0;
    thalassa::core::Scalar min_health = 0.0;
};

// Builds a 20-vs-20 skirmish as a thalassa::sim::CommandLog (spawns at
// tick 0, a move order for every unit at tick 30 converging both teams on
// a shared midline a few units apart — close enough to stay in weapon
// range once they arrive, rather than crossing all the way to the other
// side's start line and separating again) and replays it against a fresh
// SimWorld(seed) — i.e. this *is* "full replay of a fight
// from recorded commands" deliverable, run twice to prove it's
// bit-identical both times.
BattleResult run_battle_from_commands(std::uint64_t seed, int total_ticks) {
    using thalassa::sim::Command;
    using thalassa::sim::CommandLog;
    using thalassa::sim::SimWorld;
    using thalassa::sim::apply_commands;

    constexpr int kUnitsPerTeam = 20;

    std::vector<Command> spawn_commands;
    for (int i = 0; i < kUnitsPerTeam; ++i) {
        spawn_commands.push_back(
            Command::spawn_unit({static_cast<double>(i) * 2.0, 0.0, 0.0}, {}, 0, 100.0, 12.0, 15.0, 0.5));
    }
    for (int i = 0; i < kUnitsPerTeam; ++i) {
        spawn_commands.push_back(
            Command::spawn_unit({static_cast<double>(i) * 2.0, 40.0, 0.0}, {}, 1, 100.0, 12.0, 15.0, 0.5));
    }

    CommandLog log;
    for (const Command& c : spawn_commands) {
        log.add(0, c);
    }

    // Discover spawned entity ids the same way a real client would learn
    // them back from the server after issuing spawn commands: apply the
    // spawn commands to a throwaway world purely to read off the results.
    SimWorld id_discovery(seed);
    const std::vector<thalassa::core::ecs::Entity> spawned = apply_commands(id_discovery.world(), spawn_commands);

    for (int i = 0; i < kUnitsPerTeam; ++i) {
        log.add(30, Command::move_to(spawned[static_cast<std::size_t>(i)],
                                      {static_cast<double>(i) * 2.0, 17.0, 0.0}, 8.0));
    }
    for (int i = 0; i < kUnitsPerTeam; ++i) {
        log.add(30, Command::move_to(spawned[static_cast<std::size_t>(kUnitsPerTeam + i)],
                                      {static_cast<double>(i) * 2.0, 23.0, 0.0}, 8.0));
    }

    SimWorld world(seed);
    for (int t = 0; t < total_ticks; ++t) {
        const auto* commands = log.commands_at(static_cast<std::uint64_t>(t));
        if (commands != nullptr) {
            world.tick(thalassa::core::SimTick{static_cast<std::uint64_t>(t)}, *commands);
        } else {
            world.tick(thalassa::core::SimTick{static_cast<std::uint64_t>(t)});
        }
    }

    BattleResult result;
    result.survivors = static_cast<int>(world.world().component_count<Health>());
    result.casualties = 2 * kUnitsPerTeam - result.survivors;
    thalassa::core::Scalar min_health = std::numeric_limits<thalassa::core::Scalar>::max();
    world.world().each<Health>([&result, &min_health](thalassa::core::ecs::Entity e, const Health& h) {
        auto mix = [&result](std::uint64_t v) {
            result.checksum ^= v;
            result.checksum *= 1099511628211ULL;
        };
        mix(e.index());
        mix(std::bit_cast<std::uint64_t>(h.current));
        min_health = std::min(min_health, h.current);
    });
    result.min_health = (result.survivors > 0) ? min_health : 0.0;
    return result;
}

struct NetworkingDemoResult {
    std::size_t match_a_units = 0;
    std::size_t match_b_units = 0;
    bool misprediction_occurred = false;
    bool reconciliation_exact = false;
};

// Demonstrates deliverables together: MatchHost hosting
// two independent matches on one server "process," and a PredictedClient
// predicting one match ahead of confirmation, mispredicting a remote
// player's surprise move order, then reconciling to an exact match with
// the server. See tests/thalassa_net/predicted_client_tests.cpp and
// tests/thalassa_server/match_host_tests.cpp for the pinned-down
// automated versions of what this demonstrates.
NetworkingDemoResult run_networking_demo() {
    using thalassa::core::SimTick;
    using thalassa::net::PredictedClient;
    using thalassa::sim::Command;
    using thalassa::sim::apply_commands;
    using thalassa::server::MatchHost;

    constexpr std::uint64_t kSeed = 123;

    // Two independent matches hosted by one server process.
    MatchHost host;
    const auto match_a = host.create_match(kSeed);
    const auto match_b = host.create_match(kSeed + 1);
    host.submit_command(match_a, SimTick{0}, Command::spawn_unit({0.0, 0.0, 0.0}, {}, 0, 100.0, 10.0, 5.0, 1.0));
    host.submit_command(match_b, SimTick{0}, Command::spawn_unit({0.0, 0.0, 0.0}, {}, 0, 100.0, 10.0, 5.0, 1.0));
    host.submit_command(match_b, SimTick{0}, Command::spawn_unit({3.0, 0.0, 0.0}, {}, 1, 100.0, 10.0, 5.0, 1.0));
    for (int i = 0; i < 10; ++i) {
        host.tick_all();
    }

    // A client predicting a match while the "true" simulation runs
    // independently as `server_reference`, so we can show the client
    // catching back up to it after a misprediction.
    thalassa::sim::SimWorld server_reference(kSeed);
    thalassa::sim::CommandLog authoritative_log;
    const auto local_spawn = Command::spawn_unit({0.0, 0.0, 0.0}, {}, 0, 100.0, 10.0, 15.0, 0.5);
    const auto remote_spawn = Command::spawn_unit({10.0, 0.0, 0.0}, {}, 1, 100.0, 10.0, 15.0, 0.5);
    authoritative_log.add(0, local_spawn);
    authoritative_log.add(0, remote_spawn);

    PredictedClient client(kSeed);
    client.issue_command(SimTick{0}, local_spawn);
    server_reference.tick(SimTick{0}, *authoritative_log.commands_at(0));
    client.reconcile(SimTick{1}, authoritative_log);

    thalassa::sim::SimWorld id_discovery(kSeed);
    const auto spawned = apply_commands(id_discovery.world(), {local_spawn, remote_spawn});
    const auto remote_entity = spawned[1];

    for (std::uint64_t t = 1; t < 10; ++t) {
        server_reference.tick(SimTick{t});
        client.predict_tick(SimTick{t});
    }

    auto checksum_positions = [](const thalassa::sim::SimWorld& world) {
        std::uint64_t checksum = 0;
        world.world().each<Position>([&checksum](thalassa::core::ecs::Entity e, const Position& p) {
            checksum ^= (static_cast<std::uint64_t>(e.index()) << 32) ^ std::bit_cast<std::uint64_t>(p.value.x);
            checksum *= 1099511628211ULL;
        });
        return checksum;
    };

    // Tick 10: the remote player issues a move order the client hasn't
    // heard about yet — a genuine misprediction opportunity.
    authoritative_log.add(10, Command::move_to(remote_entity, {10.0, 30.0, 0.0}, 12.0));
    for (std::uint64_t t = 10; t < 20; ++t) {
        const auto* commands = authoritative_log.commands_at(t);
        if (commands != nullptr) {
            server_reference.tick(SimTick{t}, *commands);
        } else {
            server_reference.tick(SimTick{t});
        }
        client.predict_tick(SimTick{t});
    }

    NetworkingDemoResult result;
    result.match_a_units = host.match_world(match_a).world().alive_count();
    result.match_b_units = host.match_world(match_b).world().alive_count();
    result.misprediction_occurred =
        checksum_positions(client.predicted_world()) != checksum_positions(server_reference);

    client.reconcile(SimTick{20}, authoritative_log);
    result.reconciliation_exact = checksum_positions(client.predicted_world()) == checksum_positions(server_reference);

    return result;
}

}  // namespace

int main() {
    thalassa::core::log::set_min_level(thalassa::core::log::Level::Info);
    THALASSA_LOG_INFO("Thalassa sandbox: movement + spatial-index determinism smoke test (1)");

    constexpr std::uint64_t kSeed = 42;
    constexpr std::uint32_t kEntityCount = 5000;
    constexpr int kTicks = 400;

    const RunResult a = run_movement_smoke_test(kSeed, kEntityCount, kTicks);
    const RunResult b = run_movement_smoke_test(kSeed, kEntityCount, kTicks);

    THALASSA_LOG_INFO("Run A checksum: 0x{:016x}, still traveling: {}, near origin (r=20): {}",
                       a.checksum, a.still_traveling, a.spatial_hits_near_origin);
    THALASSA_LOG_INFO("Run B checksum: 0x{:016x}, still traveling: {}, near origin (r=20): {}",
                       b.checksum, b.still_traveling, b.spatial_hits_near_origin);

    if (a.checksum != b.checksum) {
        THALASSA_LOG_ERROR("DETERMINISM VIOLATION: two identical runs produced different results!");
        return EXIT_FAILURE;
    }

    THALASSA_LOG_INFO("Deterministic: {} entities over {} ticks, identical checksum across two independent runs.",
                       kEntityCount, kTicks);

    THALASSA_LOG_INFO("");
    THALASSA_LOG_INFO("Thalassa sandbox: full fight replayed from a recorded CommandLog (2)");

    constexpr int kBattleTicks = 350;
    const BattleResult battle_a = run_battle_from_commands(kSeed, kBattleTicks);
    const BattleResult battle_b = run_battle_from_commands(kSeed, kBattleTicks);

    THALASSA_LOG_INFO("Replay A checksum: 0x{:016x}, survivors: {}, casualties: {}, lowest survivor HP: {:.1f}",
                       battle_a.checksum, battle_a.survivors, battle_a.casualties, battle_a.min_health);
    THALASSA_LOG_INFO("Replay B checksum: 0x{:016x}, survivors: {}, casualties: {}, lowest survivor HP: {:.1f}",
                       battle_b.checksum, battle_b.survivors, battle_b.casualties, battle_b.min_health);

    if (battle_a.checksum != battle_b.checksum) {
        THALASSA_LOG_ERROR("DETERMINISM VIOLATION: replaying the same CommandLog twice produced different results!");
        return EXIT_FAILURE;
    }

    THALASSA_LOG_INFO("Deterministic: the same CommandLog replayed twice from a fresh SimWorld each time "
                       "produced bit-identical outcomes.");

    THALASSA_LOG_INFO("");
    THALASSA_LOG_INFO("Thalassa sandbox: multi-match hosting + client prediction/reconciliation (3)");

    const NetworkingDemoResult net_result = run_networking_demo();
    THALASSA_LOG_INFO("MatchHost: 2 independent matches hosted in one process (units alive: {} and {})",
                       net_result.match_a_units, net_result.match_b_units);
    THALASSA_LOG_INFO("PredictedClient: misprediction of remote player's surprise move order occurred: {}",
                       net_result.misprediction_occurred);
    THALASSA_LOG_INFO("PredictedClient: reconcile() restored an exact match with the server: {}",
                       net_result.reconciliation_exact);

    if (!net_result.misprediction_occurred || !net_result.reconciliation_exact) {
        THALASSA_LOG_ERROR("NETWORKING DEMO FAILED: expected a real misprediction followed by exact reconciliation.");
        return EXIT_FAILURE;
    }

    // Also demonstrate the fixed-timestep-driven HeadlessApp shape, in
    // fast-forward mode, for a few ticks.
    thalassa::server::HeadlessAppConfig config;
    config.max_ticks = 5;
    config.real_time = false;
    thalassa::server::HeadlessApp app(config);
    app.run();
    THALASSA_LOG_INFO("HeadlessApp fast-forward run complete: {} ticks simulated.",
                       app.sim_world().ticks_simulated());

    return EXIT_SUCCESS;
}
