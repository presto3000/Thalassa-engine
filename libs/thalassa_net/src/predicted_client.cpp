#include "thalassa/net/predicted_client.hpp"

#include "thalassa/core/assert.hpp"
#include "thalassa/core/serialization/byte_stream.hpp"
#include "thalassa/sim/replay.hpp"

namespace thalassa::net {

void PredictedClient::reconcile(core::SimTick authoritative_tick, const sim::CommandLog& authoritative_log) {
    THALASSA_ASSERT_MSG(core::to_u64(authoritative_tick) >= core::to_u64(confirmed_tick_),
                         "PredictedClient::reconcile: authoritative_tick moved backwards");

    // 1-2: replay confirmed_world_ forward using the AUTHORITATIVE record
    // for every tick between where it currently stands and
    // authoritative_tick — this is the actual correction, since
    // authoritative_log may contain commands (from other players) the
    // client didn't know about when it predicted through these ticks.
    while (core::to_u64(confirmed_tick_) < core::to_u64(authoritative_tick)) {
        const auto t = core::to_u64(confirmed_tick_);
        const auto* commands = authoritative_log.commands_at(t);
        if (commands != nullptr) {
            confirmed_world_.tick(core::SimTick{t}, *commands);
        } else {
            confirmed_world_.tick(core::SimTick{t});
        }
        confirmed_tick_ = confirmed_tick_ + 1;
    }

    // 3: rebuild predicted_world_ as a copy of the now-corrected
    // confirmed_world_, via the existing (already-tested) snapshot
    // serialization rather than requiring SimWorld/World to be
    // copy-constructible
    const auto target_predicted_tick = predicted_tick_;  // where local prediction had reached before this call

    core::serialization::ByteWriter writer;
    sim::save_world(confirmed_world_, writer);
    core::serialization::ByteReader reader(writer.bytes());
    sim::load_world(predicted_world_, reader);
    predicted_tick_ = confirmed_tick_;

    // 4: re-apply the client's own still-pending commands (those beyond
    // authoritative_tick) to restore its predicted-ahead position, on top
    // of the corrected foundation. Reuses predict_tick() itself — ticks
    // already folded into confirmed_world_ (and thus already reflected in
    // the fresh predicted_world_ copy) are correctly skipped since this
    // loop only runs from confirmed_tick_ (== predicted_tick_ right now)
    // up to where prediction had previously reached.
    while (core::to_u64(predicted_tick_) < core::to_u64(target_predicted_tick)) {
        predict_tick(predicted_tick_);
    }
}

}  // namespace thalassa::net
