#include "thalassa/server/game_server.hpp"

#include <algorithm>

#include "thalassa/core/assert.hpp"
#include "thalassa/sim/components.hpp"

namespace thalassa::server {

using net::BatchEntry;
using net::BatchMsg;
using net::ChannelMessage;
using net::MessageType;
using net::PeerId;

GameServer::GameServer(net::ITransport& transport, ServerConfig config, double now)
    : transport_(transport), config_(config), next_tick_time_(now) {
    THALASSA_ASSERT_MSG(std::abs(config_.tick_duration_seconds - 1.0 / 60.0) < 1e-12,
                         "GameServer: tick duration must match SimWorld's (1/60 s)");
}

core::MatchId GameServer::create_match(std::uint64_t rng_seed) {
    const core::MatchId id = host_.create_match(rng_seed);
    MatchState state;
    state.seed = rng_seed;
    matches_.emplace(id.value(), std::move(state));
    return id;
}

void GameServer::schedule_command(core::MatchId match, core::SimTick target_tick, const sim::Command& command) {
    const auto it = matches_.find(match.value());
    THALASSA_ASSERT_MSG(it != matches_.end(), "GameServer::schedule_command: unknown match");
    const std::uint64_t current = core::to_u64(host_.match_tick(match));
    it->second.scheduled.emplace(std::max(core::to_u64(target_tick), current), command);
}

void GameServer::update(double now) {
    receive_datagrams(now);

    // Handle everything clients said.
    for (auto& [peer, client] : clients_) {
        ChannelMessage m;
        while (client->channel.receive(m)) {
            handle_message(peer, *client, m, now);
        }
    }

    // Fixed-rate ticking from the supplied time.
    if (ticking_) {
        std::uint32_t ran = 0;
        while (now >= next_tick_time_ && ran < config_.max_catchup_ticks) {
            step_matches();
            next_tick_time_ += config_.tick_duration_seconds;
            ++ran;
        }
        if (now >= next_tick_time_) {
            next_tick_time_ = now;  // fell too far behind: drop the backlog instead of spiralling
        }
    } else {
        next_tick_time_ = now;
    }

    for (auto& [peer, client] : clients_) {
        (void)peer;
        if (client->phase == ClientPhase::Joined) flush_batches(*client);
        client->channel.update(now);
    }

    prune_clients(now);
}

void GameServer::receive_datagrams(double now) {
    net::Datagram d;
    while (transport_.receive(d)) {
        auto it = clients_.find(d.peer);
        if (it == clients_.end()) {
            if (clients_.size() >= config_.max_clients) {
                continue;  // table full: ignore strangers (a real transport could also send "server full")
            }
            it = clients_.emplace(d.peer, std::make_unique<Client>(transport_, d.peer, now, config_.channel)).first;
        }
        it->second->channel.on_datagram(d.payload, now);
    }
}

void GameServer::handle_message(PeerId peer, Client& client, const ChannelMessage& message, double now) {
    (void)now;
    const auto type = net::peek_type(message.payload);
    if (!type) {
        ++stats_.malformed_messages;
        return;
    }
    switch (*type) {
        case MessageType::Hello: {
            const auto hello = net::decode_hello(message.payload);
            if (!hello) { ++stats_.malformed_messages; return; }
            handle_hello(peer, client, *hello);
            break;
        }
        case MessageType::ClientCommand: {
            const auto cmd = net::decode_client_command(message.payload);
            if (!cmd) { ++stats_.malformed_messages; return; }
            if (client.phase != ClientPhase::Joined) { ++stats_.commands_rejected; return; }
            handle_client_command(client, *cmd);
            break;
        }
        case MessageType::Goodbye:
            client.leaving = true;
            break;
        default:
            ++stats_.malformed_messages;  // server->client types are not valid from a client
            break;
    }
}

void GameServer::handle_hello(PeerId peer, Client& client, const net::HelloMsg& hello) {
    (void)peer;
    if (client.phase == ClientPhase::Joined) return;  // duplicate Hello: ignore

    auto reject = [&](net::RejectReason reason) {
        (void)client.channel.send_reliable(net::encode(net::RejectMsg{reason}));
        ++stats_.joins_rejected;
        client.leaving = true;  // dropped once the Reject has had a chance to go out
    };

    if (hello.version != net::kProtocolVersion) return reject(net::RejectReason::BadVersion);
    const core::MatchId match_id{hello.match_id};
    const auto it = matches_.find(match_id.value());
    if (it == matches_.end() || !host_.has_match(match_id)) return reject(net::RejectReason::UnknownMatch);

    MatchState& match = it->second;
    client.phase = ClientPhase::Joined;
    client.match = match_id;
    client.player = core::PlayerId{next_player_id_++};
    client.team = static_cast<std::int32_t>(match.players_joined % 2);
    ++match.players_joined;
    client.history_cursor = 0;  // a joiner receives the whole history
    client.last_up_to_sent = 0;
    ++stats_.clients_joined;

    net::WelcomeMsg welcome;
    welcome.player_id = client.player.value();
    welcome.team = client.team;
    welcome.match_id = match_id.value();
    welcome.rng_seed = match.seed;
    welcome.tick_duration_seconds = config_.tick_duration_seconds;
    welcome.server_tick = core::to_u64(host_.match_tick(match_id));
    (void)client.channel.send_reliable(net::encode(welcome));
}

bool GameServer::validate_and_stamp(const Client& client, const MatchState& match, sim::Command& command) const {
    (void)match;
    switch (command.type) {
        case sim::Command::Type::SpawnUnit:
            if (!config_.allow_client_spawns) return false;
            // The server decides who owns what; never trust the client's claim.
            command.owner = client.player;
            command.team = client.team;
            if (!(command.max_health > 0.0) || command.max_health > 1.0e6) return false;
            return true;
        case sim::Command::Type::MoveTo: {
            const auto& world = host_.match_world(client.match).world();
            if (!world.is_alive(command.entity)) return false;
            if (!world.has<sim::components::Owner>(command.entity)) return false;
            if (world.get<sim::components::Owner>(command.entity).player != client.player) return false;  // not yours
            if (!(command.speed >= 0.0) || command.speed > 1.0e4) return false;
            return true;
        }
    }
    return false;
}

void GameServer::handle_client_command(Client& client, const net::ClientCommandMsg& msg) {
    const auto it = matches_.find(client.match.value());
    if (it == matches_.end()) return;
    MatchState& match = it->second;

    sim::Command command = msg.command;
    if (!validate_and_stamp(client, match, command)) {
        ++stats_.commands_rejected;
        return;
    }

    const std::uint64_t current = core::to_u64(host_.match_tick(client.match));
    const std::uint64_t target =
        std::clamp(msg.target_tick, current, current + config_.max_command_lead_ticks);  // late -> now, far -> capped
    match.scheduled.emplace(target, command);
    ++stats_.commands_accepted;
}

void GameServer::step_matches() {
    // Move commands that are due this tick into the executed history, in
    // schedule order, then run one tick of every match.
    for (auto& [raw_id, match] : matches_) {
        const core::MatchId id{raw_id};
        const std::uint64_t t = core::to_u64(host_.match_tick(id));
        while (!match.scheduled.empty() && match.scheduled.begin()->first <= t) {
            const sim::Command command = match.scheduled.begin()->second;
            match.scheduled.erase(match.scheduled.begin());
            host_.submit_command(id, core::SimTick{t}, command);
            match.history.push_back({t, command});
        }
    }
    host_.tick_all();
}

void GameServer::flush_batches(Client& client) {
    const auto it = matches_.find(client.match.value());
    if (it == matches_.end()) return;
    const MatchState& match = it->second;

    const std::uint64_t final_up_to = core::to_u64(host_.match_tick(client.match));  // ticks < this are final
    const std::size_t per_batch = net::max_batch_entries(net::ReliableChannel::kMaxMessageBytes);

    for (;;) {
        const std::size_t pending = match.history.size() - client.history_cursor;
        // While the simulation is paused nothing else will advance the clients' view, so flush any progress.
        const bool advance_due = final_up_to >= client.last_up_to_sent + config_.batch_interval_ticks ||
                                 (!ticking_ && final_up_to > client.last_up_to_sent);
        if (pending == 0 && !advance_due) return;

        BatchMsg batch;
        const std::size_t take = std::min(pending, per_batch);
        for (std::size_t i = 0; i < take; ++i) {
            const LoggedCommand& lc = match.history[client.history_cursor + i];
            batch.entries.push_back(BatchEntry{lc.tick, lc.command});
        }
        const std::size_t next = client.history_cursor + take;
        // Ticks below the first unsent entry's tick are complete; with nothing left, everything below final_up_to is.
        batch.up_to_tick = next < match.history.size() ? match.history[next].tick : final_up_to;
        batch.up_to_tick = std::max(batch.up_to_tick, client.last_up_to_sent);

        if (!client.channel.send_reliable(net::encode(batch))) return;  // window full: continue next update
        client.history_cursor = next;
        client.last_up_to_sent = batch.up_to_tick;
        if (take == 0) return;  // pure "advance" batch sent
    }
}

void GameServer::prune_clients(double now) {
    for (auto it = clients_.begin(); it != clients_.end();) {
        Client& c = *it->second;
        const bool silent = c.channel.timed_out(now);
        const bool never_joined = c.phase == ClientPhase::AwaitingHello && now - c.connected_at > config_.join_timeout_seconds;
        // A leaving client is kept one more update so its Reject/last data can be flushed, then dropped.
        // Said Goodbye: nothing more to deliver, drop immediately. Rejected join: keep it briefly so the
        // Reject can go out (and be acked), bounded by the join timeout.
        const bool said_goodbye = c.leaving && c.phase == ClientPhase::Joined;
        const bool reject_flushed = c.leaving && c.phase == ClientPhase::AwaitingHello &&
                                    (c.channel.unacked_reliable_count() == 0 ||
                                     now - c.connected_at > config_.join_timeout_seconds);
        if (silent || never_joined || said_goodbye || reject_flushed) {
            if (c.phase == ClientPhase::Joined || silent) ++stats_.clients_dropped;
            it = clients_.erase(it);
        } else {
            ++it;
        }
    }
}

sim::ReplayRecording GameServer::export_replay(core::MatchId match) const {
    const auto it = matches_.find(match.value());
    THALASSA_ASSERT_MSG(it != matches_.end(), "GameServer::export_replay: unknown match");

    sim::ReplayRecording rec;
    rec.seed = it->second.seed;
    rec.tick_duration_seconds = config_.tick_duration_seconds;
    rec.total_ticks = core::to_u64(host_.match_tick(match));
    rec.final_checksum = sim::world_checksum(host_.match_world(match));
    rec.checkpoint_interval = 0;
    for (const LoggedCommand& lc : it->second.history) rec.commands.add(lc.tick, lc.command);
    return rec;
}

}  // namespace thalassa::server
