#pragma once

#include "thalassa/core/types.hpp"
#include "thalassa/sim/commands.hpp"
#include "thalassa/sim/sim_world.hpp"

// PredictedClient: "client-side prediction +
// reconciliation" deliverable.This implements command (input) reconciliation,
// not state reconciliation:
// the client predicts ahead using its own commands (and an implicit "no
// new command" guess for everyone/everything else, which is correct
// whenever nobody issued anything new), and rolls back + replays using
// the authoritative command log whenever the server's actual record of a
// tick turns out to differ from what was guessed.

namespace thalassa::net {

class PredictedClient {
public:
    // `rng_seed` must match the seed the authoritative match was created
    // with (out-of-band agreed at match join, e.g. sent by the server
    // alongside the first snapshot) — see SimWorld's determinism
    // requirements; two SimWorlds only stay in sync if built from the
    // same seed and fed the same commands in the same tick order.
    explicit PredictedClient(std::uint64_t rng_seed) : confirmed_world_(rng_seed), predicted_world_(rng_seed) {}

    // Records a command the local player issued, targeting `target_tick`
    // (computed by the caller as "current tick + input delay"; PredictedClient doesn't pick
    // the delay itself). Does NOT apply it immediately; it takes effect
    // the next time predict_tick() reaches target_tick, exactly like it
    // will on the server.
    void issue_command(core::SimTick target_tick, sim::Command command) {
        pending_local_.add(core::to_u64(target_tick), command);
    }

    // Advances the client's own predicted view by exactly one tick,
    // applying whatever local commands were issued for this tick (and
    // implicitly guessing "nothing new" for every other entity/player,
    // which is correct unless reconcile() later says otherwise). This is
    // what lets the client render its own actions immediately instead of
    // waiting a full round-trip.
    void predict_tick(core::SimTick tick) {
        const auto* commands = pending_local_.commands_at(core::to_u64(tick));
        if (commands != nullptr) {
            predicted_world_.tick(tick, *commands);
        } else {
            predicted_world_.tick(tick);
        }
        predicted_tick_ = tick + 1;
    }

    // Reconciles against the server's authoritative record: replays
    // `confirmed_world_` forward from its current tick through
    // `authoritative_tick` using `authoritative_log` (the true command set
    // for each of those ticks, as the server recorded it — see
    // MatchHost), then rebuilds `predicted_world_` from that corrected
    // state and re-applies the client's own still-pending commands to
    // restore its predicted-ahead position.
    //
    // `authoritative_tick` must be >= this client's current confirmed
    // tick (reconciling "backwards" isn't meaningful — the server's
    // authoritative record only grows over time).
    void reconcile(core::SimTick authoritative_tick, const sim::CommandLog& authoritative_log);

    [[nodiscard]] const sim::SimWorld& predicted_world() const noexcept { return predicted_world_; }
    [[nodiscard]] const sim::SimWorld& confirmed_world() const noexcept { return confirmed_world_; }
    [[nodiscard]] core::SimTick confirmed_tick() const noexcept { return confirmed_tick_; }
    [[nodiscard]] core::SimTick predicted_tick() const noexcept { return predicted_tick_; }

private:
    sim::SimWorld confirmed_world_;
    sim::SimWorld predicted_world_;
    core::SimTick confirmed_tick_{0};
    core::SimTick predicted_tick_{0};

    // Commands this client has issued locally, keyed by their target tick.
    // that's an acceptable
    // simplification at this scope (bounded, small growth over a match).
    sim::CommandLog pending_local_;
};

}  // namespace thalassa::net
