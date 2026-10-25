#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "thalassa/core/types.hpp"
#include "thalassa/net/predicted_client.hpp"
#include "thalassa/net/protocol.hpp"
#include "thalassa/net/reliable_channel.hpp"
#include "thalassa/net/transport.hpp"
#include "thalassa/sim/commands.hpp"

// GameClient: the client end of a networked match. Joins a GameServer over
// any ITransport, receives the authoritative command stream, and keeps two
// simulations (via PredictedClient):
//   - confirmed_world(): exactly what the server has finalized, tick for tick
//   - predicted_world(): confirmed + this player's own still-unconfirmed
//     commands, run ahead so input feels instant (draw THIS one)
// When the server's record differs from the prediction (another player's
// command, a rejected or re-timed command) the next batch corrects it.
//
// Pacing: the client estimates the server's tick from the Welcome message and
// its own clock, and predicts up to "server tick + one round trip + margin" so
// that a command issued now lands on the server just before it simulates the
// tick it targets. Prediction never runs more than max_prediction_ticks past
// the last confirmed tick (if the server goes quiet, prediction stalls
// instead of running away).
//
// Like everything here it is poll-driven and takes time explicitly:
//   client.update(now);   // every frame/tick
//
// Limitation: SpawnUnit commands from clients are not predictable (entity ids
// are assigned by the server), so the server refuses them by default.

namespace thalassa::net {

struct ClientConfig {
    ChannelConfig channel;
    std::uint32_t input_delay_ticks = 2;      // extra ticks between "now" and a command's target tick
    std::uint32_t max_prediction_ticks = 90;  // 1.5 s at 60 Hz
    double safety_margin_seconds = 0.03;      // jitter allowance added to the prediction horizon
    double connect_timeout_seconds = 5.0;
};

enum class ClientState : std::uint8_t {
    Connecting,    // Hello sent, waiting for Welcome
    Joined,        // playing
    Rejected,      // server refused (see reject_reason())
    Disconnected,  // timed out, or incompatible server
};

class GameClient {
public:
    GameClient(ITransport& transport, PeerId server, std::uint32_t match_id, double now, ClientConfig config = {});

    // Pumps the network and advances prediction. Call every frame.
    void update(double now);

    // Issues a player command: predicted locally right away and sent to the
    // server. The target tick is chosen automatically (predicted_tick +
    // input_delay_ticks) and returned. nullopt if not joined, or the send
    // window is full.
    std::optional<core::SimTick> issue_command(const sim::Command& command);

    // Presentation events (attack started, projectile launched, died...) for
    // animations/VFX. Off by default; may be switched on before or after joining.
    // Events come from the confirmed world, so each is delivered exactly once, in
    // tick order, about one round trip after the predicted view shows its cause.
    void set_events_enabled(bool enabled) noexcept;
    [[nodiscard]] std::vector<sim::SimEvent> drain_events();

    // Leaves politely (the server also drops silent clients on its own).
    void disconnect(double now);

    [[nodiscard]] ClientState state() const noexcept { return state_; }
    [[nodiscard]] std::optional<RejectReason> reject_reason() const noexcept { return reject_reason_; }
    [[nodiscard]] core::PlayerId player_id() const noexcept { return player_id_; }
    [[nodiscard]] std::int32_t team() const noexcept { return team_; }

    // Valid once joined().
    [[nodiscard]] bool joined() const noexcept { return state_ == ClientState::Joined; }
    [[nodiscard]] const sim::SimWorld& predicted_world() const { return prediction_->predicted_world(); }
    [[nodiscard]] const sim::SimWorld& confirmed_world() const { return prediction_->confirmed_world(); }
    [[nodiscard]] core::SimTick confirmed_tick() const { return prediction_->confirmed_tick(); }
    [[nodiscard]] core::SimTick predicted_tick() const { return prediction_->predicted_tick(); }

    [[nodiscard]] double smoothed_rtt_seconds() const noexcept { return channel_.stats().smoothed_rtt_seconds; }
    [[nodiscard]] const ChannelStats& channel_stats() const noexcept { return channel_.stats(); }

private:
    void handle_message(const ChannelMessage& message, double now);
    void advance_prediction(double now);

    ITransport& transport_;
    PeerId server_;
    std::uint32_t match_id_;
    ClientConfig config_;
    ReliableChannel channel_;
    double started_at_;

    ClientState state_ = ClientState::Connecting;
    std::optional<RejectReason> reject_reason_;
    core::PlayerId player_id_{};
    std::int32_t team_ = 0;

    std::unique_ptr<PredictedClient> prediction_;
    sim::CommandLog authoritative_log_;
    std::uint64_t authoritative_up_to_ = 0;
    bool reconcile_needed_ = false;

    // Clock model, set at Welcome.
    std::uint64_t welcome_tick_ = 0;
    double welcome_time_ = 0.0;
    double tick_duration_ = 1.0 / 60.0;
    bool events_enabled_ = false;
};

}  // namespace thalassa::net
