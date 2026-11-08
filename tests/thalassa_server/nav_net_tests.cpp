#include <cstdint>
#include <memory>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "thalassa/net/game_client.hpp"
#include "thalassa/net/loopback_transport.hpp"
#include "thalassa/net/protocol.hpp"
#include "thalassa/server/game_server.hpp"
#include "thalassa/sim/components.hpp"
#include "thalassa/sim/nav/nav_map.hpp"
#include "thalassa/sim/replay_recording.hpp"

using namespace thalassa;
using core::PlayerId;
using core::SimTick;
using core::ecs::Entity;
using net::ClientState;
using net::GameClient;
using server::GameServer;
using sim::Command;
using sim::components::Position;
using sim::nav::NavGrid;
using sim::nav::NavMap;

namespace {

constexpr double kDt = 1.0 / 120.0;

// 40 x 40 m with a wall at x in [20,21) from y = 0 to 30; the only way across is above it.
std::shared_ptr<const NavMap> make_map(double wall_height = 30.0) {
    NavGrid g(1.0, 0.0, 0.0, 40, 40);
    (void)g.block_rect(20.0, 0.0, 21.0, wall_height);
    return NavMap::create(std::move(g), {0.4});
}

struct Rig {
    Rig() : network(7) {
        server_transport = network.create_endpoint();
        server = std::make_unique<GameServer>(*server_transport, server::ServerConfig{}, 0.0);
    }

    GameClient& add_client(core::MatchId match, std::shared_ptr<const NavMap> map) {
        client_transports.push_back(network.create_endpoint());
        net::ClientConfig cfg;
        cfg.nav_map = std::move(map);
        clients.push_back(std::make_unique<GameClient>(*client_transports.back(), server_transport->local_id(),
                                                       match.value(), now, cfg));
        return *clients.back();
    }

    void step() {
        server->update(now);
        for (auto& c : clients) c->update(now);
        network.advance(kDt);
        now += kDt;
    }
    void run(double seconds) {
        const double end = now + seconds;
        while (now < end) step();
    }
    template <typename Pred>
    bool run_until(double max_seconds, Pred&& pred) {
        const double end = now + max_seconds;
        while (now < end) {
            step();
            if (pred()) return true;
        }
        return false;
    }

    net::LoopbackNetwork network;
    std::unique_ptr<net::LoopbackTransport> server_transport;
    std::unique_ptr<GameServer> server;
    std::vector<std::unique_ptr<net::LoopbackTransport>> client_transports;
    std::vector<std::unique_ptr<GameClient>> clients;
    double now = 0.0;
};

}  // namespace

TEST_CASE("handshake: the nav map hash must match or the join is rejected", "[nav][net][server]") {
    Rig rig;
    const auto map = make_map();
    const auto match = rig.server->create_match(1, map);

    GameClient& good = rig.add_client(match, make_map());  // equal content, separate instance
    GameClient& none = rig.add_client(match, nullptr);
    GameClient& other = rig.add_client(match, make_map(25.0));
    REQUIRE(rig.run_until(5.0, [&] {
        return good.joined() && none.state() == ClientState::Rejected && other.state() == ClientState::Rejected;
    }));
    CHECK(none.reject_reason() == net::RejectReason::NavMismatch);
    CHECK(other.reject_reason() == net::RejectReason::NavMismatch);

    // The other way around: a map-less match refuses a client that brings a map.
    const auto plain = rig.server->create_match(2);
    GameClient& extra = rig.add_client(plain, map);
    GameClient& same = rig.add_client(plain, nullptr);
    REQUIRE(rig.run_until(5.0, [&] { return extra.state() == ClientState::Rejected && same.joined(); }));
    CHECK(extra.reject_reason() == net::RejectReason::NavMismatch);
}

TEST_CASE("a client with the match's map predicts and confirms routes around walls", "[nav][net][server]") {
    Rig rig;
    const auto map = make_map();
    const auto match = rig.server->create_match(1, map);
    rig.server->schedule_command(match, SimTick{0},
                                 Command::spawn_unit({10.0, 10.0, 0.0}, PlayerId{1}, 0, 100.0, 0.0, 0.0, 1.0));

    GameClient& client = rig.add_client(match, map);
    REQUIRE(rig.run_until(5.0, [&] { return client.joined(); }));

    Entity unit{};
    REQUIRE(rig.run_until(5.0, [&] {
        bool found = false;
        client.confirmed_world().world().each<Position>([&](Entity e, const Position&) { unit = e; found = true; });
        return found;
    }));
    REQUIRE(client.issue_command(Command::move_to(unit, {30.0, 10.0, 0.0}, 6.0)));

    // Prediction already routes around the wall: its y rises past the wall top before x passes 21.
    double predicted_max_y = 0.0;
    REQUIRE(rig.run_until(20.0, [&] {
        const auto& p = client.predicted_world().world().get<Position>(unit).value;
        predicted_max_y = std::max(predicted_max_y, p.y);
        return p.x > 29.9;
    }));
    CHECK(predicted_max_y > 29.0);

    // Freeze and let the confirmed world catch up: it must equal the server's, bit for bit.
    rig.server->set_ticking(false);
    const std::uint64_t target = core::to_u64(rig.server->host().match_tick(match));
    REQUIRE(rig.run_until(20.0, [&] { return core::to_u64(client.confirmed_tick()) == target; }));
    CHECK(sim::world_checksum(client.confirmed_world()) == sim::world_checksum(rig.server->host().match_world(match)));
    CHECK(client.confirmed_world().nav_hash() == map->hash());

    // The exported replay embeds the map and plays back identically.
    const sim::ReplayRecording rec = rig.server->export_replay(match);
    REQUIRE(rec.nav_map != nullptr);
    CHECK(rec.nav_map->hash() == map->hash());
    sim::ReplayPlayer player(rec);
    while (player.step_tick()) {}
    CHECK(player.final_checksum_matches());
}
