#pragma once

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

#include "thalassa/core/types.hpp"
#include "thalassa/net/udp_transport.hpp"
#include "thalassa/server/game_server.hpp"

// DedicatedServer: a GameServer on a real UDP socket with a wall-clock loop.
// This is what the headless server executable runs, and it is a library class
// (not code in main) so tests can host one in-process on 127.0.0.1.
//
// It owns one match, spawns the demo arena for `players` players at tick 0
// (3 units each: legacy / melee / ranged mix, teams facing each other), and
// optionally writes the match's replay to `replay_path` on shutdown. The
// replay comes from the server's own executed-command log, so packet loss
// on any client link cannot affect it.
//
//   auto s = DedicatedServer::create(config, &error);
//   s->run();           // blocks until stop()   (or run_for(seconds) in tests)
//   s->stop();          // thread/signal safe

namespace thalassa::server {

struct DedicatedServerConfig {
    std::string bind_address = "0.0.0.0";
    std::uint16_t port = 27015;
    std::uint64_t seed = 1;
    std::uint32_t players = 2;  // arena size: players 1..N get units
    std::filesystem::path replay_path;  // empty = do not save
    // Let players spawn units. Offered unit types: 1 = footman (melee), 2 = archer (ranged),
    // 3 = scout (legacy instant attack); each team may spawn on its own side of the arena
    // (x in [-20,-8] for even teams, [8,20] for odd), up to 12 units per player.
    bool client_spawns = false;
    ServerConfig server;
};

class DedicatedServer {
public:
    [[nodiscard]] static std::unique_ptr<DedicatedServer> create(const DedicatedServerConfig& config,
                                                                 std::string* error = nullptr);
    ~DedicatedServer();

    // One iteration of the loop with an explicit clock (seconds since start).
    void poll(double now);

    // Blocking loops. Sleep ~1 ms between iterations when idle.
    void run();
    void run_for(double seconds);
    void stop() noexcept { running_.store(false, std::memory_order_relaxed); }

    // Writes the replay now (also done automatically on destruction when replay_path is set).
    [[nodiscard]] bool save_replay() const;

    [[nodiscard]] std::uint16_t port() const noexcept { return transport_->local_port(); }
    [[nodiscard]] core::MatchId match_id() const noexcept { return match_; }
    [[nodiscard]] GameServer& game() noexcept { return *game_; }
    [[nodiscard]] const GameServer& game() const noexcept { return *game_; }
    [[nodiscard]] const net::UdpTransport& transport() const noexcept { return *transport_; }

private:
    DedicatedServer() = default;
    [[nodiscard]] double wall_now() const;

    DedicatedServerConfig config_;
    std::unique_ptr<net::UdpTransport> transport_;
    std::unique_ptr<GameServer> game_;
    core::MatchId match_{};
    std::atomic<bool> running_{false};
    double start_seconds_ = 0.0;
    bool saved_ = false;
};

}  // namespace thalassa::server
