#include <bit>
#include <cstdint>

#include <catch2/catch_test_macros.hpp>

#include "thalassa/net/predicted_client.hpp"
#include "thalassa/sim/commands.hpp"
#include "thalassa/sim/components.hpp"

using thalassa::core::SimTick;
using thalassa::core::ecs::Entity;
using thalassa::net::PredictedClient;
using thalassa::sim::Command;
using thalassa::sim::CommandLog;
using thalassa::sim::SimWorld;
using thalassa::sim::apply_commands;
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

void tick_both(SimWorld& server, const CommandLog& authoritative_log, PredictedClient& client, std::uint64_t t) {
    const auto* cmds = authoritative_log.commands_at(t);
    if (cmds != nullptr) {
        server.tick(SimTick{t}, *cmds);
    } else {
        server.tick(SimTick{t});
    }
    client.predict_tick(SimTick{t});
}

}  // namespace

TEST_CASE("PredictedClient with no misprediction matches the server exactly after reconcile", "[predicted_client]") {
    constexpr std::uint64_t kSeed = 10;

    // Solo-player scenario: every command in the match is the client's own,
    // so "guess nothing new" is always correct — no bootstrap step needed.
    SimWorld server(kSeed);
    CommandLog authoritative_log;
    PredictedClient client(kSeed);

    const auto spawn = Command::spawn_unit({0.0, 0.0, 0.0}, {}, 0, 100.0, 10.0, 5.0, 1.0);
    authoritative_log.add(0, spawn);
    client.issue_command(SimTick{0}, spawn);

    for (std::uint64_t t = 0; t < 10; ++t) {
        tick_both(server, authoritative_log, client, t);
    }
    client.reconcile(SimTick{10}, authoritative_log);

    REQUIRE(checksum_of(client.confirmed_world()) == checksum_of(server));
    REQUIRE(checksum_of(client.predicted_world()) == checksum_of(server));
}

TEST_CASE("PredictedClient mispredicts another player's command, then reconcile corrects it exactly",
          "[predicted_client]") {
    constexpr std::uint64_t kSeed = 20;

    SimWorld server(kSeed);
    CommandLog authoritative_log;
    PredictedClient client(kSeed);

    // Two players spawn at tick 0. This is match-start roster information,
    // not something the client predicts — in a real system the server
    // tells every client the initial roster before simulation begins, so
    // we model that here as an immediate reconcile() through tick 0,
    // before any local prediction happens. This isolates the test's actual
    // point (misprediction of a LATER unknown command) from a trivial
    // "client doesn't know the other player exists at all" difference.
    const auto local_spawn = Command::spawn_unit({0.0, 0.0, 0.0}, {}, 0, 100.0, 10.0, 5.0, 1.0);
    const auto remote_spawn = Command::spawn_unit({50.0, 0.0, 0.0}, {}, 1, 100.0, 10.0, 5.0, 1.0);
    authoritative_log.add(0, local_spawn);
    authoritative_log.add(0, remote_spawn);
    client.issue_command(SimTick{0}, local_spawn);

    server.tick(SimTick{0}, *authoritative_log.commands_at(0));
    client.reconcile(SimTick{1}, authoritative_log);
    REQUIRE(checksum_of(client.confirmed_world()) == checksum_of(server));
    REQUIRE(checksum_of(client.predicted_world()) == checksum_of(server));

    SimWorld id_discovery(kSeed);
    const auto spawned = apply_commands(id_discovery.world(), {local_spawn, remote_spawn});
    const Entity remote_entity = spawned[1];

    // Ticks 1-4: nothing new happens on either side; predictions match.
    for (std::uint64_t t = 1; t < 5; ++t) {
        tick_both(server, authoritative_log, client, t);
    }
    client.reconcile(SimTick{5}, authoritative_log);
    REQUIRE(checksum_of(client.confirmed_world()) == checksum_of(server));

    // Tick 5: the REMOTE player issues a move command the client has no
    // way to know about yet. The server applies it; the client, predicting
    // ahead, guesses "nothing new" (the only thing it can do) and is wrong.
    const auto remote_move = Command::move_to(remote_entity, {50.0, 40.0, 0.0}, 20.0);
    authoritative_log.add(5, remote_move);

    for (std::uint64_t t = 5; t < 10; ++t) {
        tick_both(server, authoritative_log, client, t);  // client's predict_tick has NOT been told about remote_move
    }

    // The misprediction must be real and observable before we prove
    // reconcile() fixes it — otherwise this test would trivially pass even
    // with a broken reconcile().
    REQUIRE(checksum_of(client.predicted_world()) != checksum_of(server));

    client.reconcile(SimTick{10}, authoritative_log);  // now the client learns about remote_move too

    REQUIRE(checksum_of(client.confirmed_world()) == checksum_of(server));
    REQUIRE(checksum_of(client.predicted_world()) == checksum_of(server));
}

TEST_CASE("PredictedClient keeps predicting ahead of confirmed_tick after reconcile", "[predicted_client]") {
    constexpr std::uint64_t kSeed = 30;
    PredictedClient client(kSeed);
    CommandLog authoritative_log;

    const auto spawn = Command::spawn_unit({0.0, 0.0, 0.0}, {}, 0, 100.0, 10.0, 5.0, 1.0);
    authoritative_log.add(0, spawn);
    client.issue_command(SimTick{0}, spawn);

    for (std::uint64_t t = 0; t < 5; ++t) {
        client.predict_tick(SimTick{t});
    }
    REQUIRE(thalassa::core::to_u64(client.predicted_tick()) == 5);

    client.reconcile(SimTick{3}, authoritative_log);  // server only confirms through tick 3 so far

    REQUIRE(thalassa::core::to_u64(client.confirmed_tick()) == 3);
    // Prediction should still be ahead at tick 5 — reconcile() must not
    // have thrown away the client's own not-yet-confirmed local ticks.
    REQUIRE(thalassa::core::to_u64(client.predicted_tick()) == 5);
}

TEST_CASE("Reconciliation is deterministic across two independent identical client/server pairs",
          "[predicted_client][determinism]") {
    auto run = []() {
        constexpr std::uint64_t kSeed = 999;
        SimWorld server(kSeed);
        CommandLog authoritative_log;
        PredictedClient client(kSeed);

        const auto local_spawn = Command::spawn_unit({0.0, 0.0, 0.0}, {}, 0, 100.0, 10.0, 5.0, 1.0);
        const auto remote_spawn = Command::spawn_unit({20.0, 0.0, 0.0}, {}, 1, 100.0, 10.0, 5.0, 1.0);
        authoritative_log.add(0, local_spawn);
        authoritative_log.add(0, remote_spawn);
        client.issue_command(SimTick{0}, local_spawn);

        server.tick(SimTick{0}, *authoritative_log.commands_at(0));
        client.reconcile(SimTick{1}, authoritative_log);

        SimWorld id_discovery(kSeed);
        const auto spawned = apply_commands(id_discovery.world(), {local_spawn, remote_spawn});
        authoritative_log.add(7, Command::move_to(spawned[1], {20.0, 15.0, 0.0}, 5.0));

        for (std::uint64_t t = 1; t < 15; ++t) {
            tick_both(server, authoritative_log, client, t);
        }
        client.reconcile(SimTick{15}, authoritative_log);
        return checksum_of(client.predicted_world());
    };

    REQUIRE(run() == run());
}
