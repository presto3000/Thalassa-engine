#include <cstdint>
#include <memory>
#include <optional>
#include <set>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "thalassa/net/game_client.hpp"
#include "thalassa/net/loopback_transport.hpp"
#include "thalassa/net/protocol.hpp"
#include "thalassa/server/game_server.hpp"
#include "thalassa/sim/components.hpp"
#include "thalassa/sim/replay_recording.hpp"

using namespace thalassa;
using core::MatchId;
using core::PlayerId;
using core::SimTick;
using core::ecs::Entity;
using net::ClientState;
using net::GameClient;
using net::LinkConditions;
using net::LoopbackNetwork;
using server::GameServer;
using server::ServerConfig;
using sim::Command;
using sim::components::Health;
using sim::components::Owner;
using sim::components::Position;

namespace {

constexpr double kDt = 1.0 / 120.0;

// A server plus any number of clients on one simulated network.
struct World {
    explicit World(std::uint64_t seed, const LinkConditions& link = {}, ServerConfig cfg = {})
        : network(seed) {
        network.set_default_conditions(link);
        server_transport = network.create_endpoint();  // endpoint 0
        server = std::make_unique<GameServer>(*server_transport, cfg, 0.0);
    }

    GameClient& add_client(MatchId match, net::ClientConfig cfg = {}) {
        client_transports.push_back(network.create_endpoint());
        clients.push_back(std::make_unique<GameClient>(*client_transports.back(), server_transport->local_id(),
                                                       match.value(), now, cfg));
        return *clients.back();
    }

    void step() {
        server->update(now);
        for (std::size_t i = 0; i < clients.size(); ++i) {
            if (!silenced.contains(i)) clients[i]->update(now);
        }
        network.advance(kDt);
        now += kDt;
    }

    void run(double seconds) {
        const double end = now + seconds;
        while (now < end) step();
    }

    bool run_until_joined(double max_seconds = 10.0) {
        const double end = now + max_seconds;
        while (now < end) {
            step();
            bool all = true;
            for (auto& c : clients) all &= c->joined();
            if (all) return true;
        }
        return false;
    }

    // Stops the simulation and waits until every client's confirmed world has caught up to the server's tick.
    bool settle(MatchId match, double max_seconds = 30.0) {
        server->set_ticking(false);
        const double end = now + max_seconds;
        const std::uint64_t target = core::to_u64(server->host().match_tick(match));
        while (now < end) {
            step();
            bool all = true;
            for (std::size_t i = 0; i < clients.size(); ++i) {
                if (silenced.contains(i) || !clients[i]->joined()) continue;
                all &= core::to_u64(clients[i]->confirmed_tick()) == target;
            }
            if (all) return true;
        }
        return false;
    }

    LoopbackNetwork network;
    std::unique_ptr<net::LoopbackTransport> server_transport;
    std::unique_ptr<GameServer> server;
    std::vector<std::unique_ptr<net::LoopbackTransport>> client_transports;
    std::vector<std::unique_ptr<GameClient>> clients;
    std::set<std::size_t> silenced;  // clients that "crashed": no longer updated
    double now = 0.0;
};

// Initial arena, spawned by the server at tick 0: 3 units per player. Players join
// in order, so they are PlayerId 1 and 2. Mixed legacy / melee / ranged heroes.
void spawn_arena(GameServer& server, MatchId match) {
    for (int i = 0; i < 3; ++i) {
        const double y = i * 3.0;
        Command a = Command::spawn_unit({-12.0, y, 0.0}, PlayerId{1}, 0, 120.0, 12.0, 15.0, 0.8);
        Command b = Command::spawn_unit({12.0, y + 1.0, 0.0}, PlayerId{2}, 1, 120.0, 12.0, 15.0, 0.8);
        if (i == 1) { a = a.as_melee(0.2); b = b.as_ranged(0.25, 30.0); }
        server.schedule_command(match, SimTick{0}, a);
        server.schedule_command(match, SimTick{0}, b);
    }
}

std::vector<Entity> units_of(const sim::SimWorld& w, PlayerId p) {
    std::vector<Entity> out;
    w.world().each<Owner>([&](Entity e, const Owner& o) { if (o.player == p) out.push_back(e); });
    return out;
}

std::uint64_t checksum(const sim::SimWorld& w) { return sim::world_checksum(w); }

// Both players order their units to meet in the middle, a few seconds apart.
void play_fight(World& w, GameClient& a, GameClient& b) {
    for (const Entity e : units_of(a.confirmed_world(), a.player_id())) {
        REQUIRE(a.issue_command(Command::move_to(e, {-1.0, 4.0, 0.0}, 6.0)));
    }
    w.run(0.4);
    for (const Entity e : units_of(b.confirmed_world(), b.player_id())) {
        REQUIRE(b.issue_command(Command::move_to(e, {1.0, 4.0, 0.0}, 6.0)));
    }
    w.run(6.0);
}

}  // namespace

TEST_CASE("handshake: client joins, gets identity, seed and the initial history", "[net][server]") {
    World w(1, {.latency_seconds = 0.02});
    const MatchId m = w.server->create_match(777);
    spawn_arena(*w.server, m);
    GameClient& a = w.add_client(m);

    REQUIRE(w.run_until_joined());
    REQUIRE(a.state() == ClientState::Joined);
    REQUIRE(a.player_id() == PlayerId{1});
    REQUIRE(a.team() == 0);
    REQUIRE(w.server->client_count() == 1);

    w.run(0.5);
    REQUIRE(units_of(a.confirmed_world(), PlayerId{1}).size() == 3);
    REQUIRE(units_of(a.confirmed_world(), PlayerId{2}).size() == 3);
}

TEST_CASE("handshake: two players get different ids and teams", "[net][server]") {
    World w(2, {.latency_seconds = 0.02});
    const MatchId m = w.server->create_match(1);
    GameClient& a = w.add_client(m);
    GameClient& b = w.add_client(m);
    REQUIRE(w.run_until_joined());
    REQUIRE(a.player_id() != b.player_id());
    REQUIRE(a.team() != b.team());
}

TEST_CASE("handshake: unknown match and wrong protocol version are rejected", "[net][server]") {
    World w(3, {.latency_seconds = 0.01});
    const MatchId m = w.server->create_match(1);
    (void)m;

    GameClient& lost = w.add_client(MatchId{4242});
    w.run(1.0);
    REQUIRE(lost.state() == ClientState::Rejected);
    REQUIRE(lost.reject_reason() == net::RejectReason::UnknownMatch);

    // A hand-built Hello with a different protocol version.
    auto raw_transport = w.network.create_endpoint();
    net::ReliableChannel raw(*raw_transport, w.server_transport->local_id(), w.now);
    REQUIRE(raw.send_reliable(net::encode(net::HelloMsg{999, m.value()})));
    std::optional<net::RejectMsg> rejected;
    for (int i = 0; i < 300 && !rejected; ++i) {
        raw.update(w.now);
        w.server->update(w.now);
        w.network.advance(kDt);
        w.now += kDt;
        net::Datagram d;
        while (raw_transport->receive(d)) raw.on_datagram(d.payload, w.now);
        net::ChannelMessage cm;
        while (raw.receive(cm)) rejected = net::decode_reject(cm.payload);
    }
    REQUIRE(rejected);
    REQUIRE(rejected->reason == net::RejectReason::BadVersion);
    REQUIRE(w.server->stats().joins_rejected == 2);
}

TEST_CASE("two players fight; both clients converge bit-exactly to the server (clean link)", "[net][server]") {
    World w(4, {.latency_seconds = 0.03});
    const MatchId m = w.server->create_match(2024);
    spawn_arena(*w.server, m);
    GameClient& a = w.add_client(m);
    GameClient& b = w.add_client(m);
    REQUIRE(w.run_until_joined());
    w.run(0.5);

    play_fight(w, a, b);
    REQUIRE(w.settle(m));

    const auto replay = w.server->export_replay(m);
    REQUIRE(replay.total_ticks > 300);
    REQUIRE(checksum(a.confirmed_world()) == replay.final_checksum);
    REQUIRE(checksum(b.confirmed_world()) == replay.final_checksum);
    REQUIRE(w.server->stats().commands_accepted == 6);
    REQUIRE(w.server->stats().commands_rejected == 0);
}

TEST_CASE("convergence survives a nasty network (25% loss, jitter, duplication)", "[net][server]") {
    World w(5, {.latency_seconds = 0.04, .jitter_seconds = 0.03, .loss_probability = 0.25,
                .duplicate_probability = 0.15});
    const MatchId m = w.server->create_match(31337);
    spawn_arena(*w.server, m);
    GameClient& a = w.add_client(m);
    GameClient& b = w.add_client(m);
    REQUIRE(w.run_until_joined(30.0));
    w.run(1.0);

    play_fight(w, a, b);
    REQUIRE(w.settle(m, 120.0));

    const auto replay = w.server->export_replay(m);
    REQUIRE(checksum(a.confirmed_world()) == replay.final_checksum);
    REQUIRE(checksum(b.confirmed_world()) == replay.final_checksum);
    REQUIRE(w.network.stats().dropped > 100);  // the loss was real
}

TEST_CASE("the server's replay is unaffected by packet loss and plays back bit-identically", "[net][server][replay]") {
    World w(6, {.latency_seconds = 0.04, .jitter_seconds = 0.03, .loss_probability = 0.3});
    const MatchId m = w.server->create_match(99);
    spawn_arena(*w.server, m);
    GameClient& a = w.add_client(m);
    GameClient& b = w.add_client(m);
    REQUIRE(w.run_until_joined(30.0));
    play_fight(w, a, b);
    w.server->set_ticking(false);

    auto replay = w.server->export_replay(m);

    // Through the file format and back, as a downloaded replay would be.
    core::serialization::ByteWriter bytes;
    replay.serialize(bytes);
    sim::ReplayRecording loaded;
    core::serialization::ByteReader reader(bytes.bytes());
    REQUIRE(loaded.deserialize(reader));

    sim::ReplayPlayer player(loaded);
    while (player.step_tick()) {}
    REQUIRE(player.final_checksum_matches());
}

TEST_CASE("prediction: a player's own command shows up before the server confirms it", "[net][server]") {
    World w(7, {.latency_seconds = 0.08});  // 160 ms round trip
    const MatchId m = w.server->create_match(5);
    spawn_arena(*w.server, m);
    GameClient& a = w.add_client(m);
    REQUIRE(w.run_until_joined());
    w.run(1.0);

    const Entity unit = units_of(a.confirmed_world(), a.player_id())[0];
    const double start_x = a.predicted_world().world().get<Position>(unit).value.x;
    REQUIRE(a.issue_command(Command::move_to(unit, {10.0, 0.0, 0.0}, 12.0)));

    w.run(0.12);  // less than a round trip: the server's answer cannot have arrived yet
    const double predicted_x = a.predicted_world().world().get<Position>(unit).value.x;
    const double confirmed_x = a.confirmed_world().world().get<Position>(unit).value.x;
    REQUIRE(predicted_x > start_x + 0.5);          // already moving on screen
    REQUIRE(confirmed_x < predicted_x);            // server-confirmed state lags behind
    REQUIRE(core::to_u64(a.predicted_tick()) > core::to_u64(a.confirmed_tick()));

    w.run(0.5);  // let the command reach the server and execute
    REQUIRE(w.settle(m));
    REQUIRE(a.confirmed_world().world().get<Position>(unit).value.x > start_x + 0.5);  // and then it agrees
}

TEST_CASE("authority: a player cannot move another player's unit, nor spawn units", "[net][server]") {
    World w(8, {.latency_seconds = 0.02});
    const MatchId m = w.server->create_match(5);
    spawn_arena(*w.server, m);
    GameClient& a = w.add_client(m);
    GameClient& b = w.add_client(m);
    REQUIRE(w.run_until_joined());
    w.run(0.5);

    const Entity victim = units_of(a.confirmed_world(), a.player_id())[0];
    const auto before = a.confirmed_world().world().get<Position>(victim).value;

    REQUIRE(b.issue_command(Command::move_to(victim, {50.0, 50.0, 0.0}, 20.0)));  // B tries to steal A's unit
    REQUIRE(b.issue_command(Command::spawn_unit({0, 0, 0}, PlayerId{2}, 1, 100.0, 50.0, 30.0, 0.1)));
    w.run(1.0);

    REQUIRE(w.server->stats().commands_rejected == 2);
    REQUIRE(w.server->stats().commands_accepted == 0);
    REQUIRE(w.settle(m));
    REQUIRE(a.confirmed_world().world().get<Position>(victim).value == before);
    REQUIRE(units_of(a.confirmed_world(), PlayerId{2}).size() == 3);  // no extra unit appeared
    // B's own prediction of the illegal move is corrected away once the server's record catches up.
    REQUIRE(b.confirmed_world().world().get<Position>(victim).value == before);
    REQUIRE(b.predicted_world().world().get<Position>(victim).value == before);
}

TEST_CASE("a player who joins late replays the history and converges", "[net][server]") {
    World w(9, {.latency_seconds = 0.03, .loss_probability = 0.1});
    const MatchId m = w.server->create_match(2024);
    spawn_arena(*w.server, m);
    GameClient& a = w.add_client(m);
    GameClient& b = w.add_client(m);
    REQUIRE(w.run_until_joined(20.0));
    play_fight(w, a, b);

    GameClient& late = w.add_client(m);  // joins mid-match, after the whole fight
    w.run(3.0);
    REQUIRE(late.joined());
    REQUIRE(w.settle(m, 60.0));

    const auto replay = w.server->export_replay(m);
    REQUIRE(checksum(late.confirmed_world()) == replay.final_checksum);
}

TEST_CASE("a large command history streams in several batches", "[net][server]") {
    World w(10, {.latency_seconds = 0.02, .loss_probability = 0.1});
    const MatchId m = w.server->create_match(1);
    for (int i = 0; i < 300; ++i) {
        w.server->schedule_command(m, SimTick{static_cast<std::uint64_t>(i / 10)},
                                   Command::spawn_unit({static_cast<double>(i), 0, 0}, PlayerId{1}, 0, 50.0, 1.0, 1.0, 1.0));
    }
    GameClient& a = w.add_client(m);
    REQUIRE(w.run_until_joined());
    w.run(2.0);
    REQUIRE(w.settle(m, 60.0));
    REQUIRE(units_of(a.confirmed_world(), PlayerId{1}).size() == 300);
    REQUIRE(checksum(a.confirmed_world()) == w.server->export_replay(m).final_checksum);
}

TEST_CASE("matches are isolated from each other", "[net][server]") {
    World w(11, {.latency_seconds = 0.02});
    const MatchId m1 = w.server->create_match(1);
    const MatchId m2 = w.server->create_match(2);
    spawn_arena(*w.server, m1);  // only match 1 has units
    GameClient& c1 = w.add_client(m1);
    GameClient& c2 = w.add_client(m2);
    REQUIRE(w.run_until_joined());
    w.run(1.0);
    REQUIRE(units_of(c1.confirmed_world(), PlayerId{1}).size() == 3);
    REQUIRE(units_of(c2.confirmed_world(), PlayerId{1}).empty());
    REQUIRE(units_of(c2.confirmed_world(), PlayerId{2}).empty());
}

TEST_CASE("a client that stops responding is dropped by the server", "[net][server]") {
    ServerConfig cfg;
    cfg.channel.timeout_seconds = 2.0;
    World w(12, {.latency_seconds = 0.02}, cfg);
    const MatchId m = w.server->create_match(1);
    w.add_client(m);
    GameClient& b = w.add_client(m);
    (void)b;
    REQUIRE(w.run_until_joined());
    REQUIRE(w.server->client_count() == 2);

    w.silenced.insert(0);  // client 0 crashes: no more updates, no keepalives
    w.run(1.0);
    REQUIRE(w.server->client_count() == 2);  // not yet
    w.run(3.0);
    REQUIRE(w.server->client_count() == 1);
    REQUIRE(w.server->stats().clients_dropped == 1);
}

TEST_CASE("a client says goodbye and is removed", "[net][server]") {
    World w(13, {.latency_seconds = 0.02});
    const MatchId m = w.server->create_match(1);
    GameClient& a = w.add_client(m);
    REQUIRE(w.run_until_joined());
    a.disconnect(w.now);
    w.run(1.0);
    REQUIRE(w.server->client_count() == 0);
}

TEST_CASE("garbage from a stranger never joins anyone and is forgotten", "[net][server]") {
    ServerConfig cfg;
    cfg.join_timeout_seconds = 1.0;
    World w(14, {}, cfg);
    (void)w.server->create_match(1);

    auto stranger = w.network.create_endpoint();
    for (int i = 0; i < 20; ++i) {
        std::vector<std::byte> junk(static_cast<std::size_t>(10 + i * 7), static_cast<std::byte>(0xA5 + i));
        REQUIRE(stranger->send(w.server_transport->local_id(), junk));
        w.step();
    }
    REQUIRE(w.server->stats().clients_joined == 0);
    w.run(2.0);
    REQUIRE(w.server->client_count() == 0);  // the half-open peer expired
}

TEST_CASE("the server caps how many strangers it tracks", "[net][server]") {
    ServerConfig cfg;
    cfg.max_clients = 4;
    World w(15, {}, cfg);
    (void)w.server->create_match(1);
    std::vector<std::unique_ptr<net::LoopbackTransport>> strangers;
    for (int i = 0; i < 20; ++i) {
        strangers.push_back(w.network.create_endpoint());
        std::vector<std::byte> junk(16, std::byte{0x11});
        REQUIRE(strangers.back()->send(w.server_transport->local_id(), junk));
    }
    w.run(0.1);
    REQUIRE(w.server->client_count() <= 4);
}

TEST_CASE("stress: many seeds of a terrible network all converge exactly", "[net][server][stress]") {
    for (std::uint64_t seed = 200; seed < 212; ++seed) {
        INFO("seed " << seed);
        World w(seed, {.latency_seconds = 0.05, .jitter_seconds = 0.05, .loss_probability = 0.35,
                       .duplicate_probability = 0.25});
        const MatchId m = w.server->create_match(seed);
        spawn_arena(*w.server, m);
        GameClient& a = w.add_client(m);
        GameClient& b = w.add_client(m);
        REQUIRE(w.run_until_joined(60.0));
        w.run(1.0);
        play_fight(w, a, b);
        REQUIRE(w.settle(m, 180.0));
        const auto replay = w.server->export_replay(m);
        REQUIRE(checksum(a.confirmed_world()) == replay.final_checksum);
        REQUIRE(checksum(b.confirmed_world()) == replay.final_checksum);
    }
}

TEST_CASE("events: a client's animation events are exactly the replay's, each delivered once", "[net][server][events]") {
    World w(21, {.latency_seconds = 0.04, .jitter_seconds = 0.03, .loss_probability = 0.25, .duplicate_probability = 0.05});
    const MatchId m = w.server->create_match(7);
    spawn_arena(*w.server, m);
    GameClient& a = w.add_client(m);
    GameClient& b = w.add_client(m);
    a.set_events_enabled(true);  // before joining: applied when the world is created
    REQUIRE(w.run_until_joined(30.0));
    play_fight(w, a, b);
    REQUIRE(w.settle(m));

    // Drained in several slices, as a game would every frame; the result must be the same.
    std::vector<sim::SimEvent> got = a.drain_events();
    CHECK(b.drain_events().empty());  // b never asked for events

    const sim::ReplayRecording recording = w.server->export_replay(m);  // the player keeps a reference
    sim::ReplayPlayer player(recording);
    while (player.step_tick()) {}
    const std::vector<sim::SimEvent> want = player.drain_events();

    REQUIRE_FALSE(want.empty());
    bool has_attack = false;
    for (const auto& e : want) has_attack |= e.type == sim::SimEvent::Type::AttackStarted;
    CHECK(has_attack);

    REQUIRE(got.size() == want.size());
    for (std::size_t i = 0; i < got.size(); ++i) {
        CHECK(got[i].type == want[i].type);
        CHECK(got[i].tick == want[i].tick);
        CHECK(got[i].source == want[i].source);
        CHECK(got[i].target == want[i].target);
        CHECK(got[i].value == want[i].value);
    }
    // Nothing is delivered twice.
    CHECK(a.drain_events().empty());
}

// ---------------------------------------------------------------------------
// Client spawn requests
// ---------------------------------------------------------------------------
namespace {

ServerConfig spawn_config(std::uint32_t cap = 4) {
    ServerConfig c;
    auto& p = c.client_spawns;
    p.enabled = true;
    p.templates.push_back({1, Command::spawn_unit({}, PlayerId{}, 0, 200.0, 20.0, 3.0, 0.9).as_melee(0.2)});
    p.templates.push_back({2, Command::spawn_unit({}, PlayerId{}, 0, 80.0, 9.0, 12.0, 1.0).as_ranged(0.3, 30.0)});
    p.areas.push_back({0, {-20.0, -5.0, -1.0}, {-8.0, 15.0, 1.0}});
    p.areas.push_back({1, {8.0, -5.0, -1.0}, {20.0, 15.0, 1.0}});
    p.max_units_per_player = cap;
    p.min_ticks_between_spawns = 10;
    return c;
}

}  // namespace

TEST_CASE("client spawn: the server picks the stats; every client converges, even on a bad network",
          "[net][server][spawn]") {
    World w(31, {.latency_seconds = 0.03, .jitter_seconds = 0.02, .loss_probability = 0.25}, spawn_config());
    const MatchId m = w.server->create_match(5);
    GameClient& a = w.add_client(m);
    GameClient& b = w.add_client(m);
    REQUIRE(w.run_until_joined(30.0));

    // A cheating request: claims another owner/team and absurd stats. Only type + position count.
    Command cheat = Command::request_spawn(1, {-12.0, 2.0, 0.0});
    cheat.owner = PlayerId{2};
    cheat.team = 1;
    cheat.max_health = 1.0e5;
    cheat.attack_damage = 1.0e5;
    REQUIRE(a.issue_command(cheat));
    REQUIRE(b.issue_command(Command::request_spawn(2, {12.0, 3.0, 0.0})));
    w.run(1.0);
    REQUIRE(w.settle(m));

    const auto mine = units_of(a.confirmed_world(), a.player_id());
    REQUIRE(mine.size() == 1);
    const auto& wa = a.confirmed_world().world();
    CHECK(wa.get<sim::components::Owner>(mine[0]).player == a.player_id());
    CHECK(wa.get<sim::components::Team>(mine[0]).id == a.team());
    CHECK(wa.get<Health>(mine[0]).max == 200.0);  // the template's, not 1e5
    CHECK(wa.get<sim::components::CombatStats>(mine[0]).attack_damage == 20.0);
    REQUIRE(wa.has<sim::components::AttackProfile>(mine[0]));
    CHECK(wa.get<sim::components::AttackProfile>(mine[0]).kind == sim::components::AttackProfile::kMelee);

    const auto theirs = units_of(a.confirmed_world(), b.player_id());
    REQUIRE(theirs.size() == 1);
    CHECK(wa.get<Health>(theirs[0]).max == 80.0);

    const auto server_sum = checksum(w.server->host().match_world(m));
    CHECK(checksum(a.confirmed_world()) == server_sum);
    CHECK(checksum(b.confirmed_world()) == server_sum);
    CHECK(w.server->stats().commands_accepted == 2);

    // The fresh unit can be ordered by its owner (and only by its owner).
    w.server->set_ticking(true);  // settle() froze the simulation
    REQUIRE(a.issue_command(Command::move_to(mine[0], {-10.0, 8.0, 0.0}, 6.0)));
    REQUIRE(b.issue_command(Command::move_to(mine[0], {0.0, 0.0, 0.0}, 6.0)));  // not b's unit
    w.run(2.0);
    REQUIRE(w.settle(m));
    const auto& pos = a.confirmed_world().world().get<Position>(mine[0]).value;
    CHECK(pos.y > 4.0);  // walked toward y = 8, was not dragged to the origin
    CHECK(w.server->stats().commands_rejected == 1);
    CHECK(checksum(a.confirmed_world()) == checksum(w.server->host().match_world(m)));

    // The server's own replay of it all is exact.
    const sim::ReplayRecording recording = w.server->export_replay(m);
    sim::ReplayPlayer player(recording);
    while (player.step_tick()) {}
    CHECK(player.final_checksum_matches());
}

TEST_CASE("client spawn: bad requests are rejected and change nothing", "[net][server][spawn]") {
    World w(32, {.latency_seconds = 0.01}, spawn_config(/*cap=*/3));
    const MatchId m = w.server->create_match(5);
    GameClient& a = w.add_client(m);  // player 1, team 0: may spawn in x in [-20,-8]
    REQUIRE(w.run_until_joined());

    auto try_spawn = [&](const Command& c) {
        REQUIRE(a.issue_command(c));
        w.run(0.4);  // longer than the rate limit
    };
    auto my_units = [&] { return units_of(w.server->host().match_world(m), a.player_id()).size(); };

    try_spawn(Command::request_spawn(99, {-12.0, 2.0, 0.0}));   // unknown template
    try_spawn(Command::request_spawn(0, {-12.0, 2.0, 0.0}));    // "custom stats"
    try_spawn(Command::request_spawn(1, {12.0, 2.0, 0.0}));     // enemy team's area
    try_spawn(Command::request_spawn(1, {-12.0, 50.0, 0.0}));   // outside every area
    Command custom = Command::spawn_unit({-12.0, 2.0, 0.0}, PlayerId{1}, 0, 1.0e5, 1.0e5, 50.0, 0.1);
    try_spawn(custom);                                           // type 0 with its own stats
    CHECK(my_units() == 0);
    CHECK(w.server->stats().commands_rejected == 5);
    CHECK(w.server->stats().commands_accepted == 0);

    // Rate limit: two requests in the same instant, only the first counts.
    REQUIRE(a.issue_command(Command::request_spawn(1, {-12.0, 1.0, 0.0})));
    REQUIRE(a.issue_command(Command::request_spawn(1, {-12.0, 2.0, 0.0})));
    w.run(0.5);
    CHECK(my_units() == 1);

    // Cap of 3: the fourth request is refused no matter how patient we are.
    for (int i = 0; i < 4; ++i) try_spawn(Command::request_spawn(2, {-14.0, 3.0 + i, 0.0}));
    CHECK(my_units() == 3);

    REQUIRE(w.settle(m));
    CHECK(checksum(a.confirmed_world()) == checksum(w.server->host().match_world(m)));
}

TEST_CASE("client spawn: off by default, and a flood cannot exceed the cap", "[net][server][spawn]") {
    {
        World w(33, {.latency_seconds = 0.01});  // default config: spawning disabled
        const MatchId m = w.server->create_match(5);
        GameClient& a = w.add_client(m);
        REQUIRE(w.run_until_joined());
        REQUIRE(a.issue_command(Command::request_spawn(1, {-12.0, 2.0, 0.0})));
        w.run(0.5);
        CHECK(units_of(w.server->host().match_world(m), a.player_id()).empty());
        CHECK(w.server->stats().commands_rejected == 1);
    }
    {
        ServerConfig cfg = spawn_config(/*cap=*/5);
        cfg.client_spawns.min_ticks_between_spawns = 0;  // only the cap protects us
        World w(34, {.latency_seconds = 0.02}, cfg);
        const MatchId m = w.server->create_match(5);
        GameClient& a = w.add_client(m);
        REQUIRE(w.run_until_joined());
        for (int i = 0; i < 40; ++i) {
            (void)a.issue_command(Command::request_spawn(1, {-12.0, 2.0, 0.0}));  // window may fill: fine
        }
        w.run(2.0);
        CHECK(units_of(w.server->host().match_world(m), a.player_id()).size() == 5);
    }
}
