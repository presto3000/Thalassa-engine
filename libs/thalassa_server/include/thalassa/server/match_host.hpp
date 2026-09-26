#pragma once

#include <unordered_map>
#include <vector>

#include "thalassa/core/assert.hpp"
#include "thalassa/core/types.hpp"
#include "thalassa/sim/commands.hpp"
#include "thalassa/sim/sim_world.hpp"

// MatchHost: "support for multiple concurrent game
// instances on one server process" deliverable. Owns a set of independent
// matches — each its own SimWorld, CommandLog, and tick counter, sharing
// no mutable state with any other match and advances all of them together
// once per server loop iteration.
// This sits above SimWorld/HeadlessApp, not in place of them: HeadlessApp
// (thalassa/server/headless_app.hpp) remains the right tool for a single
// always-on world driven by wall-clock time.
// MatchHost is for a server process hosting several independent matches at
// once, each advanced by an explicit tick_all() call — pairing MatchHost
// with a FixedTimestep-driven loop (see HeadlessApp for the pattern) is
// how a real dedicated server would combine the two.

namespace thalassa::server {

class MatchHost {
public:
    // Creates a new match with its own SimWorld(rng_seed) and an empty
    // CommandLog, and returns its id. Match ids are never reused within
    // one MatchHost's lifetime, even after destroy_match() — so a stale
    // MatchId from a destroyed match is detectably invalid (has_match()
    // returns false) rather than silently aliasing a later match, the
    // same generation-safety idea as core::ecs::Entity.
    [[nodiscard]] core::MatchId create_match(std::uint64_t rng_seed) {
        const core::MatchId id{next_id_++};
        matches_.emplace(id.value(), Match{sim::SimWorld(rng_seed), sim::CommandLog{}, core::SimTick{0}});
        return id;
    }

    void destroy_match(core::MatchId id) { matches_.erase(id.value()); }

    [[nodiscard]] bool has_match(core::MatchId id) const { return matches_.contains(id.value()); }

    // Records `command` as authoritative for `target_tick` in the given
    // match. If `target_tick` has already been simulated (i.e. it's not
    // strictly in the future relative to the match's current tick), it's
    // clamped to the next tick that hasn't run yet and the command is
    // still applied there
    void submit_command(core::MatchId id, core::SimTick target_tick, sim::Command command) {
        Match& match = match_at(id);
        const std::uint64_t requested = core::to_u64(target_tick);
        const std::uint64_t current = core::to_u64(match.current_tick);
        const std::uint64_t scheduled_tick = requested >= current ? requested : current;
        match.log.add(scheduled_tick, command);
    }

    // Advances every match by exactly one tick, applying whatever
    // commands were submitted for that match's current tick. Matches are
    // ticked independently of each other — the order this method visits
    // them in has no effect on any individual match's outcome, since
    // matches share no state, only on the (irrelevant) order they'd
    // appear in if you were, say, logging each one as it ticks.
    void tick_all() {
        for (auto& [id, match] : matches_) {
            (void)id;
            const auto t = core::to_u64(match.current_tick);
            const auto* commands = match.log.commands_at(t);
            if (commands != nullptr) {
                match.world.tick(match.current_tick, *commands);
            } else {
                match.world.tick(match.current_tick);
            }
            match.current_tick = match.current_tick + 1;
        }
    }

    [[nodiscard]] const sim::SimWorld& match_world(core::MatchId id) const { return match_at(id).world; }
    [[nodiscard]] const sim::CommandLog& match_log(core::MatchId id) const { return match_at(id).log; }
    // "Next tick to run" convention — consistent with
    // thalassa::net::PredictedClient's confirmed_tick()/predicted_tick(),
    // so a client reconciling against this match's log passes this value
    // straight through as `authoritative_tick`.
    [[nodiscard]] core::SimTick match_tick(core::MatchId id) const { return match_at(id).current_tick; }

    [[nodiscard]] std::size_t match_count() const noexcept { return matches_.size(); }

    // Returns every currently-hosted match id. Order is not meaningful
    // (backed by an unordered_map) — callers that need a stable order
    // should sort this themselves; nothing about ticking or simulating
    // depends on it (see tick_all()'s comment).
    [[nodiscard]] std::vector<core::MatchId> match_ids() const {
        std::vector<core::MatchId> ids;
        ids.reserve(matches_.size());
        for (const auto& [id, match] : matches_) {
            (void)match;
            ids.push_back(core::MatchId{id});
        }
        return ids;
    }

private:
    struct Match {
        sim::SimWorld world;
        sim::CommandLog log;
        core::SimTick current_tick;
    };

    [[nodiscard]] Match& match_at(core::MatchId id) {
        const auto it = matches_.find(id.value());
        THALASSA_ASSERT_MSG(it != matches_.end(), "MatchHost: unknown MatchId");
        return it->second;
    }
    [[nodiscard]] const Match& match_at(core::MatchId id) const {
        const auto it = matches_.find(id.value());
        THALASSA_ASSERT_MSG(it != matches_.end(), "MatchHost: unknown MatchId");
        return it->second;
    }

    std::unordered_map<core::MatchId::ValueType, Match> matches_;
    core::MatchId::ValueType next_id_ = 0;
};

}  // namespace thalassa::server
