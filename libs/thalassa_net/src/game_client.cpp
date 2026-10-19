#include "thalassa/net/game_client.hpp"

#include <algorithm>
#include <cmath>

namespace thalassa::net {

GameClient::GameClient(ITransport& transport, PeerId server, std::uint32_t match_id, double now,
                       ClientConfig config)
    : transport_(transport),
      server_(server),
      match_id_(match_id),
      config_(config),
      channel_(transport, server, now, config.channel),
      started_at_(now) {
    (void)channel_.send_reliable(encode(HelloMsg{kProtocolVersion, match_id_}));
}

void GameClient::update(double now) {
    if (state_ == ClientState::Rejected || state_ == ClientState::Disconnected) {
        return;
    }

    Datagram d;
    while (transport_.receive(d)) {
        if (d.peer == server_) channel_.on_datagram(d.payload, now);
    }

    ChannelMessage m;
    while (channel_.receive(m)) {
        handle_message(m, now);
        if (state_ == ClientState::Rejected || state_ == ClientState::Disconnected) {
            return;
        }
    }

    if (reconcile_needed_ && prediction_ != nullptr) {
        // One reconcile per update, after all batches were absorbed.
        prediction_->reconcile(core::SimTick{authoritative_up_to_}, authoritative_log_);
        reconcile_needed_ = false;
    }

    advance_prediction(now);
    channel_.update(now);

    if (channel_.timed_out(now) ||
        (state_ == ClientState::Connecting && now - started_at_ > config_.connect_timeout_seconds)) {
        state_ = ClientState::Disconnected;
    }
}

void GameClient::handle_message(const ChannelMessage& message, double now) {
    const auto type = peek_type(message.payload);
    if (!type) return;

    switch (*type) {
        case MessageType::Welcome: {
            const auto w = decode_welcome(message.payload);
            if (!w || state_ != ClientState::Connecting) return;
            // Our simulation worlds tick at SimWorld's default rate.
            if (std::abs(w->tick_duration_seconds - 1.0 / 60.0) > 1e-12) {
                state_ = ClientState::Disconnected;  // incompatible server
                return;
            }
            player_id_ = core::PlayerId{w->player_id};
            team_ = w->team;
            tick_duration_ = w->tick_duration_seconds;
            welcome_tick_ = w->server_tick;
            welcome_time_ = now;
            prediction_ = std::make_unique<PredictedClient>(w->rng_seed);
            state_ = ClientState::Joined;
            break;
        }
        case MessageType::Reject: {
            const auto r = decode_reject(message.payload);
            if (!r) return;
            reject_reason_ = r->reason;
            state_ = ClientState::Rejected;
            break;
        }
        case MessageType::Batch: {
            const auto b = decode_batch(message.payload);
            if (!b || state_ != ClientState::Joined) return;
            for (const BatchEntry& e : b->entries) {
                authoritative_log_.add(e.tick, e.command);
            }
            // up_to only moves forward (reliable + ordered, but be defensive).
            if (b->up_to_tick > authoritative_up_to_) {
                authoritative_up_to_ = b->up_to_tick;
                reconcile_needed_ = true;
            }
            break;
        }
        default:
            break;  // client->server types are meaningless here: ignore
    }
}

void GameClient::advance_prediction(double now) {
    if (state_ != ClientState::Joined || prediction_ == nullptr) return;

    // horizon = server tick at Welcome + elapsed ticks + one round trip + margin
    const double rtt = channel_.stats().smoothed_rtt_seconds;
    const double horizon_ticks =
        static_cast<double>(welcome_tick_) +
        ((now - welcome_time_) + rtt + config_.safety_margin_seconds) / tick_duration_;
    const auto horizon = static_cast<std::uint64_t>(std::max(horizon_ticks, 0.0));

    const std::uint64_t cap = core::to_u64(prediction_->confirmed_tick()) + config_.max_prediction_ticks;
    const std::uint64_t target = std::min(horizon, cap);
    while (core::to_u64(prediction_->predicted_tick()) < target) {
        prediction_->predict_tick(prediction_->predicted_tick());
    }
}

std::optional<core::SimTick> GameClient::issue_command(const sim::Command& command) {
    if (state_ != ClientState::Joined || prediction_ == nullptr) return std::nullopt;

    const core::SimTick target = prediction_->predicted_tick() + config_.input_delay_ticks;
    if (!channel_.send_reliable(encode(ClientCommandMsg{core::to_u64(target), command}))) {
        return std::nullopt;  // window full: the server is not acking
    }
    prediction_->issue_command(target, command);
    return target;
}

void GameClient::disconnect(double now) {
    if (state_ == ClientState::Joined || state_ == ClientState::Connecting) {
        (void)channel_.send_reliable(encode_goodbye());
        channel_.update(now);  // flush the Goodbye right away
    }
    state_ = ClientState::Disconnected;
}

}  // namespace thalassa::net
