#include "thalassa/server/dedicated_server.hpp"

#include <chrono>
#include <thread>

#include "thalassa/core/log.hpp"
#include "thalassa/sim/commands.hpp"
#include "thalassa/sim/replay_file.hpp"

namespace thalassa::server {

namespace {

double steady_seconds() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

// Players 1..N, three units each, teams on opposite sides (player i: team i-1).
void spawn_arena(GameServer& server, core::MatchId match, std::uint32_t players) {
    for (std::uint32_t p = 1; p <= players; ++p) {
        const double x = (p % 2 == 1) ? -12.0 : 12.0;
        const double x_offset = static_cast<double>((p - 1) / 2) * 3.0 * ((p % 2 == 1) ? -1.0 : 1.0);
        for (int i = 0; i < 3; ++i) {
            sim::Command c = sim::Command::spawn_unit({x + x_offset, i * 3.0 + (p % 2 == 0 ? 1.0 : 0.0), 0.0},
                                                      core::PlayerId{p}, static_cast<std::int32_t>(p - 1), 120.0,
                                                      12.0, 15.0, 0.8);
            if (i == 1) c = (p % 2 == 1) ? c.as_melee(0.2) : c.as_ranged(0.25, 30.0);
            server.schedule_command(match, core::SimTick{0}, c);
        }
    }
}

void configure_client_spawns(ServerConfig& server, std::uint32_t players) {
    ClientSpawnPolicy& p = server.client_spawns;
    p.enabled = true;
    p.templates.push_back({1, sim::Command::spawn_unit({}, core::PlayerId{}, 0, 140.0, 14.0, 2.5, 0.9).as_melee(0.25)});
    p.templates.push_back({2, sim::Command::spawn_unit({}, core::PlayerId{}, 0, 90.0, 11.0, 15.0, 1.0).as_ranged(0.3, 30.0)});
    p.templates.push_back({3, sim::Command::spawn_unit({}, core::PlayerId{}, 0, 100.0, 12.0, 15.0, 0.8)});
    for (std::uint32_t team = 0; team < players; ++team) {
        const bool left = team % 2 == 0;
        p.areas.push_back({static_cast<std::int32_t>(team),
                           {left ? -20.0 : 8.0, -5.0, -1.0}, {left ? -8.0 : 20.0, 15.0, 1.0}});
    }
    p.max_units_per_player = 12;
    p.min_ticks_between_spawns = 10;
}

}  // namespace

std::unique_ptr<DedicatedServer> DedicatedServer::create(const DedicatedServerConfig& config, std::string* error) {
    std::unique_ptr<DedicatedServer> s(new DedicatedServer());
    s->config_ = config;

    net::UdpConfig udp;
    udp.bind_address = config.bind_address;
    udp.port = config.port;
    s->transport_ = net::UdpTransport::create(udp, error);
    if (s->transport_ == nullptr) return nullptr;

    s->start_seconds_ = steady_seconds();
    ServerConfig server_config = config.server;
    if (config.client_spawns) configure_client_spawns(server_config, config.players);
    s->game_ = std::make_unique<GameServer>(*s->transport_, server_config, 0.0);
    s->match_ = s->game_->create_match(config.seed);
    spawn_arena(*s->game_, s->match_, config.players);
    return s;
}

DedicatedServer::~DedicatedServer() {
    if (!config_.replay_path.empty() && !saved_ && game_ != nullptr) (void)save_replay();
}

double DedicatedServer::wall_now() const { return steady_seconds() - start_seconds_; }

void DedicatedServer::poll(double now) { game_->update(now); }

void DedicatedServer::run() {
    running_.store(true, std::memory_order_relaxed);
    while (running_.load(std::memory_order_relaxed)) {
        poll(wall_now());
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void DedicatedServer::run_for(double seconds) {
    running_.store(true, std::memory_order_relaxed);
    const double end = wall_now() + seconds;
    while (running_.load(std::memory_order_relaxed) && wall_now() < end) {
        poll(wall_now());
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

bool DedicatedServer::save_replay() const {
    if (config_.replay_path.empty()) return false;
    const bool ok = sim::save_replay_file(config_.replay_path, game_->export_replay(match_));
    // `saved_` is only bookkeeping for the destructor.
    const_cast<DedicatedServer*>(this)->saved_ = ok;
    return ok;
}

}  // namespace thalassa::server
