#pragma once

#include <map>
#include <memory>
#include <vector>

#include "thalassa/core/types.hpp"
#include "thalassa/net/protocol.hpp"
#include "thalassa/net/reliable_channel.hpp"
#include "thalassa/net/transport.hpp"
#include "thalassa/server/match_host.hpp"
#include "thalassa/sim/commands.hpp"
#include "thalassa/sim/replay_recording.hpp"

// GameServer: the dedicated-server end of networked play. Sits on any
// ITransport (loopback in tests, UDP later), hosts matches through
// MatchHost, and speaks the protocol in thalassa/net/protocol.hpp:
//
//   - accepts Hello, assigns a player id and team, replies Welcome
//   - validates every client command (see "Authority" below) and schedules it
//     for its target tick
//   - ticks all matches at a fixed rate from the time you pass to update()
//   - streams each client the authoritative command log in batches (a late
//     joiner receives the whole history, so it converges by replaying it)
//   - drops silent clients
//
// Authority: clients are untrusted. A MoveTo is accepted only for an entity
// the sending player OWNS. Client SpawnUnit is refused unless
// ClientSpawnPolicy::enabled, and even then the client only REQUESTS a unit:
// it names a server-defined template (Command::unit_type) and a position; the
// server rejects unknown templates, positions outside the team's spawn area,
// spawns faster than min_ticks_between_spawns, and spawns beyond
// max_units_per_player (living units + spawns still waiting for their tick),
// then overwrites owner, team and ALL stats from the template. A client can
// therefore never choose health, damage, team or owner. Target ticks are
// clamped to [now, now + max lead].
// Spawning the initial units is the server's job: schedule_command().
//
// Every command the simulation ever runs is recorded in the match's command
// log at the moment its tick runs, so the log is exactly what the match
// executed no matter what packets were lost. export_replay() turns it into a
// ReplayRecording that replays bit-identically.

namespace thalassa::server {

// A unit a client may ask for. `stats` is a complete spawn command (health, damage,
// range, cooldown, attack kind/windup/projectile speed); its position, owner and team
// are ignored and replaced per request.
struct UnitTemplate {
    std::uint32_t id = 0;  // what clients put in Command::unit_type (must be non-zero)
    sim::Command stats;
};

// Where a team may spawn units (axis-aligned box, meters, Thalassa coordinates).
struct SpawnArea {
    std::int32_t team = 0;
    core::math::Vec3 min{0.0, 0.0, 0.0};
    core::math::Vec3 max{0.0, 0.0, 0.0};
};

struct ClientSpawnPolicy {
    bool enabled = false;
    std::vector<UnitTemplate> templates;
    // If empty, clients may spawn anywhere. If not empty, a request must lie inside an
    // area of the SENDER's team (a team without an area cannot spawn).
    std::vector<SpawnArea> areas;
    std::uint32_t max_units_per_player = 50;
    std::uint32_t min_ticks_between_spawns = 10;  // per player (10 ticks = 6 spawns/second at 60 Hz)
};

struct ServerConfig {
    double tick_duration_seconds = 1.0 / 60.0;  // must equal SimWorld's tick duration (1/60)
    std::uint32_t batch_interval_ticks = 3;     // how often "up to tick N is final" is sent when idle
    std::size_t max_clients = 64;
    std::uint32_t max_command_lead_ticks = 120;  // furthest future tick a client may target
    ClientSpawnPolicy client_spawns;
    double join_timeout_seconds = 5.0;  // peers that never complete a Hello are dropped
    std::uint32_t max_catchup_ticks = 8;  // ticks run per update() at most (avoids a spiral after a stall)
    net::ChannelConfig channel;
};

struct ServerStats {
    std::uint64_t clients_joined = 0;
    std::uint64_t clients_dropped = 0;
    std::uint64_t commands_accepted = 0;
    std::uint64_t commands_rejected = 0;
    std::uint64_t malformed_messages = 0;
    std::uint64_t joins_rejected = 0;
};

class GameServer {
public:
    GameServer(net::ITransport& transport, ServerConfig config, double now);

    // Creates a match. Players join it by id.
    [[nodiscard]] core::MatchId create_match(std::uint64_t rng_seed);

    // Server-authoritative command (e.g. initial spawns), scheduled for
    // `target_tick` (clamped up to the match's next tick if already past).
    void schedule_command(core::MatchId match, core::SimTick target_tick, const sim::Command& command);

    // Pumps the network, ticks matches (using `now`), streams batches, drops dead clients.
    void update(double now);

    // Pause/resume simulation ticking (networking keeps running). Handy for tests.
    void set_ticking(bool ticking) noexcept { ticking_ = ticking; }

    [[nodiscard]] const MatchHost& host() const noexcept { return host_; }
    [[nodiscard]] std::size_t client_count() const noexcept { return clients_.size(); }
    [[nodiscard]] const ServerStats& stats() const noexcept { return stats_; }

    // Everything the match has executed so far, as a replay (see class comment).
    [[nodiscard]] sim::ReplayRecording export_replay(core::MatchId match) const;

private:
    struct LoggedCommand {
        std::uint64_t tick;
        sim::Command command;
    };
    struct MatchState {
        std::uint64_t seed = 0;
        std::vector<LoggedCommand> history;                 // executed commands, ascending tick, execution order
        std::multimap<std::uint64_t, sim::Command> scheduled;  // future commands (insertion order kept per tick)
        std::uint32_t players_joined = 0;
    };
    enum class ClientPhase : std::uint8_t { AwaitingHello, Joined };
    struct Client {
        Client(net::ITransport& t, net::PeerId peer, double now, const net::ChannelConfig& cc)
            : channel(t, peer, now, cc), connected_at(now) {}
        net::ReliableChannel channel;
        double connected_at;
        ClientPhase phase = ClientPhase::AwaitingHello;
        core::MatchId match{};
        core::PlayerId player{};
        std::int32_t team = 0;
        std::size_t history_cursor = 0;  // next history entry to send
        std::uint64_t last_up_to_sent = 0;
        bool leaving = false;
        bool has_spawned = false;
        std::uint64_t last_spawn_tick = 0;  // match tick of the last accepted spawn request
    };

    void receive_datagrams(double now);
    void handle_message(net::PeerId peer, Client& client, const net::ChannelMessage& message, double now);
    void handle_hello(net::PeerId peer, Client& client, const net::HelloMsg& hello);
    void handle_client_command(Client& client, const net::ClientCommandMsg& msg);
    [[nodiscard]] bool validate_and_stamp(Client& client, const MatchState& match, sim::Command& command);
    [[nodiscard]] bool validate_spawn_request(Client& client, const MatchState& match, sim::Command& command);
    void step_matches();
    void flush_batches(Client& client);
    void prune_clients(double now);

    net::ITransport& transport_;
    ServerConfig config_;
    MatchHost host_;
    std::map<core::MatchId::ValueType, MatchState> matches_;  // ordered: deterministic iteration
    std::map<net::PeerId, std::unique_ptr<Client>> clients_;
    core::PlayerId::ValueType next_player_id_ = 1;
    ServerStats stats_;

    bool ticking_ = true;
    double next_tick_time_;
};

}  // namespace thalassa::server
