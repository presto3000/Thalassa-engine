#include <chrono>
#include <filesystem>
#include <memory>
#include <random>
#include <thread>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "thalassa/net/game_client.hpp"
#include "thalassa/net/udp_transport.hpp"
#include "thalassa/server/dedicated_server.hpp"
#include "thalassa/sim/components.hpp"
#include "thalassa/sim/replay_file.hpp"
#include "thalassa/sim/replay_recording.hpp"

using namespace thalassa;
using core::PlayerId;
using core::ecs::Entity;
using net::GameClient;
using server::DedicatedServer;
using server::DedicatedServerConfig;
using sim::Command;

namespace {

// Drops a fraction of datagrams in both directions on top of a REAL UDP socket,
// so the reliability layer is exercised over actual OS sockets with loss.
class LossyUdp final : public net::ITransport {
public:
    LossyUdp(net::UdpTransport& inner, double loss, std::uint32_t seed) : inner_(inner), loss_(loss), rng_(seed) {}
    [[nodiscard]] net::PeerId local_id() const noexcept override { return inner_.local_id(); }
    [[nodiscard]] bool send(net::PeerId to, std::span<const std::byte> p) override {
        if (drop()) return true;
        return inner_.send(to, p);
    }
    [[nodiscard]] bool receive(net::Datagram& out) override {
        while (inner_.receive(out)) {
            if (!drop()) return true;
        }
        return false;
    }

private:
    bool drop() { return std::uniform_real_distribution<double>(0.0, 1.0)(rng_) < loss_; }
    net::UdpTransport& inner_;
    double loss_;
    std::mt19937 rng_;
};

struct Rig {
    explicit Rig(std::filesystem::path replay = {}) {
        DedicatedServerConfig cfg;
        cfg.bind_address = "127.0.0.1";
        cfg.port = 0;  // any free port
        cfg.seed = 99;
        cfg.players = 2;
        cfg.replay_path = std::move(replay);
        std::string err;
        server = DedicatedServer::create(cfg, &err);
        INFO(err);
        REQUIRE(server != nullptr);
        start = std::chrono::steady_clock::now();
    }

    net::UdpTransport& add_socket() {
        net::UdpConfig u;
        u.bind_address = "127.0.0.1";
        sockets.push_back(net::UdpTransport::create(u));
        REQUIRE(sockets.back() != nullptr);
        server_peer.push_back(sockets.back()->add_peer("127.0.0.1", server->port()));
        return *sockets.back();
    }

    GameClient& add_client(net::ITransport& t, std::size_t socket_index) {
        clients.push_back(std::make_unique<GameClient>(t, server_peer[socket_index], server->match_id().value(), now()));
        return *clients.back();
    }

    double now() const { return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count(); }

    void pump() {
        const double t = now();
        server->poll(t);
        for (auto& c : clients) c->update(t);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    void run(double seconds) {
        const double end = now() + seconds;
        while (now() < end) pump();
    }

    bool run_until(double max_seconds, auto&& predicate) {
        const double end = now() + max_seconds;
        while (now() < end) {
            pump();
            if (predicate()) return true;
        }
        return false;
    }

    bool all_joined() const {
        for (const auto& c : clients) if (!c->joined()) return false;
        return true;
    }

    // Freezes the simulation and waits for every client to catch up to the server's tick.
    bool settle() {
        server->game().set_ticking(false);
        const auto target = core::to_u64(server->game().host().match_tick(server->match_id()));
        return run_until(20.0, [&] {
            for (const auto& c : clients) if (core::to_u64(c->confirmed_tick()) != target) return false;
            return true;
        });
    }

    std::unique_ptr<DedicatedServer> server;
    std::vector<std::unique_ptr<net::UdpTransport>> sockets;
    std::vector<net::PeerId> server_peer;
    std::vector<std::unique_ptr<GameClient>> clients;
    std::chrono::steady_clock::time_point start;
};

std::vector<Entity> units_of(const sim::SimWorld& w, PlayerId p) {
    std::vector<Entity> out;
    w.world().each<sim::components::Owner>([&](Entity e, const sim::components::Owner& o) {
        if (o.player == p) out.push_back(e);
    });
    return out;
}

}  // namespace

TEST_CASE("dedicated server: two clients play over real UDP sockets and converge", "[server][udp][e2e]") {
    Rig rig;
    GameClient& a = rig.add_client(rig.add_socket(), 0);
    GameClient& b = rig.add_client(rig.add_socket(), 1);
    REQUIRE(rig.run_until(5.0, [&] { return rig.all_joined(); }));
    CHECK(a.player_id() != b.player_id());
    CHECK(rig.server->game().client_count() == 2);

    rig.run(0.3);
    REQUIRE(units_of(a.confirmed_world(), a.player_id()).size() == 3);
    for (const Entity e : units_of(a.confirmed_world(), a.player_id())) {
        REQUIRE(a.issue_command(Command::move_to(e, {-1.0, 4.0, 0.0}, 6.0)));
    }
    rig.run(0.3);
    for (const Entity e : units_of(b.confirmed_world(), b.player_id())) {
        REQUIRE(b.issue_command(Command::move_to(e, {1.0, 4.0, 0.0}, 6.0)));
    }
    rig.run(2.5);

    REQUIRE(rig.settle());
    const auto server_sum = sim::world_checksum(rig.server->game().host().match_world(rig.server->match_id()));
    CHECK(sim::world_checksum(a.confirmed_world()) == server_sum);
    CHECK(sim::world_checksum(b.confirmed_world()) == server_sum);
    CHECK(rig.server->game().stats().commands_accepted == 6);
    CHECK(rig.server->game().stats().commands_rejected == 0);
}

TEST_CASE("dedicated server: a lossy client on a real socket still converges, and the replay is exact",
          "[server][udp][e2e]") {
    const auto path = std::filesystem::temp_directory_path() / "thalassa_dedicated_test.thr";
    std::filesystem::remove(path);
    std::uint64_t server_sum = 0;
    {
        Rig rig(path);
        net::UdpTransport& sa = rig.add_socket();
        net::UdpTransport& sb = rig.add_socket();
        LossyUdp lossy_a(sa, 0.25, 1);
        LossyUdp lossy_b(sb, 0.30, 2);
        GameClient& a = rig.add_client(lossy_a, 0);
        GameClient& b = rig.add_client(lossy_b, 1);
        REQUIRE(rig.run_until(10.0, [&] { return rig.all_joined(); }));

        rig.run(0.3);
        for (const Entity e : units_of(a.confirmed_world(), a.player_id())) {
            REQUIRE(a.issue_command(Command::move_to(e, {-1.0, 4.0, 0.0}, 6.0)));
        }
        for (const Entity e : units_of(b.confirmed_world(), b.player_id())) {
            REQUIRE(b.issue_command(Command::move_to(e, {1.0, 4.0, 0.0}, 6.0)));
        }
        rig.run(3.0);

        REQUIRE(rig.settle());
        server_sum = sim::world_checksum(rig.server->game().host().match_world(rig.server->match_id()));
        CHECK(sim::world_checksum(a.confirmed_world()) == server_sum);
        CHECK(sim::world_checksum(b.confirmed_world()) == server_sum);
        CHECK(rig.server->game().stats().commands_accepted == 6);
        REQUIRE(rig.server->save_replay());
    }

    sim::ReplayRecording rec;
    REQUIRE(sim::load_replay_file(path, rec));
    sim::ReplayPlayer player(rec);
    while (!player.finished()) player.step_tick();
    CHECK_FALSE(player.desynced());
    CHECK(player.final_checksum_matches());
    CHECK(sim::world_checksum(player.sim()) == server_sum);
    std::filesystem::remove(path);
}

TEST_CASE("dedicated server: a late joiner over UDP catches up from the history", "[server][udp][e2e]") {
    Rig rig;
    GameClient& a = rig.add_client(rig.add_socket(), 0);
    REQUIRE(rig.run_until(5.0, [&] { return a.joined(); }));
    rig.run(0.3);
    for (const Entity e : units_of(a.confirmed_world(), a.player_id())) {
        REQUIRE(a.issue_command(Command::move_to(e, {0.0, 4.0, 0.0}, 6.0)));
    }
    rig.run(2.0);

    GameClient& late = rig.add_client(rig.add_socket(), 1);
    REQUIRE(rig.run_until(5.0, [&] { return late.joined(); }));
    rig.run(0.5);
    REQUIRE(rig.settle());
    CHECK(sim::world_checksum(late.confirmed_world()) ==
          sim::world_checksum(rig.server->game().host().match_world(rig.server->match_id())));
}

TEST_CASE("dedicated server: garbage from a stranger does not disturb a running match", "[server][udp][e2e]") {
    Rig rig;
    GameClient& a = rig.add_client(rig.add_socket(), 0);
    REQUIRE(rig.run_until(5.0, [&] { return a.joined(); }));

    net::UdpTransport& stranger = rig.add_socket();
    std::mt19937 rng(5);
    for (int i = 0; i < 200; ++i) {
        std::vector<std::byte> junk(1 + rng() % 200);
        for (auto& b : junk) b = static_cast<std::byte>(rng());
        (void)stranger.send(rig.server_peer.back(), junk);
        if (i % 20 == 0) rig.run(0.01);
    }
    rig.run(0.5);
    CHECK(a.joined());
    CHECK(rig.server->game().client_count() <= 2);
    REQUIRE(rig.settle());
    CHECK(sim::world_checksum(a.confirmed_world()) ==
          sim::world_checksum(rig.server->game().host().match_world(rig.server->match_id())));
}

TEST_CASE("dedicated server: binding an occupied port fails cleanly", "[server][udp]") {
    Rig rig;
    DedicatedServerConfig cfg;
    cfg.bind_address = "127.0.0.1";
    cfg.port = rig.server->port();
    std::string err;
    CHECK(DedicatedServer::create(cfg, &err) == nullptr);
    CHECK_FALSE(err.empty());
}

TEST_CASE("dedicated server: --client-spawns lets a player add units over real UDP", "[server][udp][e2e][spawn]") {
    DedicatedServerConfig cfg;
    cfg.bind_address = "127.0.0.1";
    cfg.port = 0;
    cfg.client_spawns = true;
    std::string err;
    auto server = DedicatedServer::create(cfg, &err);
    REQUIRE(server != nullptr);

    net::UdpConfig u;
    u.bind_address = "127.0.0.1";
    auto sock = net::UdpTransport::create(u);
    REQUIRE(sock != nullptr);
    const auto peer = sock->add_peer("127.0.0.1", server->port());
    const auto t0 = std::chrono::steady_clock::now();
    auto now = [&] { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); };
    GameClient client(*sock, peer, server->match_id().value(), now());
    auto run_until = [&](double seconds, auto&& done) {
        const double end = now() + seconds;
        while (now() < end) {
            server->poll(now());
            client.update(now());
            if (done()) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return false;
    };

    REQUIRE(run_until(5.0, [&] { return client.joined(); }));
    REQUIRE(client.player_id() == PlayerId{1});
    const std::size_t before = units_of(client.confirmed_world(), PlayerId{1}).size();  // 3 from the arena
    REQUIRE(client.issue_command(Command::request_spawn(2, {-12.0, 4.0, 0.0})));       // archer, own side
    REQUIRE(client.issue_command(Command::request_spawn(2, {12.0, 4.0, 0.0})));         // enemy side: refused
    REQUIRE(run_until(5.0, [&] { return units_of(client.confirmed_world(), PlayerId{1}).size() == before + 1; }));
    (void)run_until(0.5, [] { return false; });
    CHECK(units_of(client.confirmed_world(), PlayerId{1}).size() == before + 1);
    CHECK(server->game().stats().commands_rejected == 1);
}
