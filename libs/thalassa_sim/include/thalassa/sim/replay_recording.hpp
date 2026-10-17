#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "thalassa/core/serialization/byte_stream.hpp"
#include "thalassa/sim/commands.hpp"
#include "thalassa/sim/sim_world.hpp"

// Basic replay system: record a WHOLE match from tick 0, play it back later.
//
// A match is a pure function of (seed, tick duration, per-tick commands), so
// a replay stores exactly that plus integrity data:
//   - seed, tick_duration_seconds, total_ticks
//   - the full CommandLog
//   - checkpoints: (tick, state hash) every `checkpoint_interval` ticks, so
//     playback can detect a desync (different build/platform/bug) at the
//     first diverging checkpoint rather than only at the end
//   - final_checksum: hash of the complete world state after the last tick
//
// Speed control lives ONLY in ReplayPlayer's real-time -> tick-count mapping;
// the simulation itself always runs fixed ticks, so any speed (0.25x, 4x,
// paused, seeking) yields bit-identical state at any given tick.
//
// This is deliberately the "record everything from the start" version.
// Future features (seek via periodic full snapshots, reverse playback,
// mid-game join) build on `checkpoints` and sim::save_world.

namespace thalassa::sim {

inline constexpr std::uint32_t kReplayMagic = 0x50525448;  // "HTRP"
inline constexpr std::uint32_t kReplayVersion = 2;  // v2: Command carries attack kind/windup/projectile speed

// FNV-1a 64 over the complete serialized world state (save_world bytes).
[[nodiscard]] std::uint64_t world_checksum(const SimWorld& world);

struct ReplayCheckpoint {
    std::uint64_t tick = 0;  // ticks simulated when the hash was taken
    std::uint64_t checksum = 0;
};

struct ReplayRecording {
    std::uint64_t seed = 0;
    core::Scalar tick_duration_seconds = 1.0 / 60.0;
    std::uint64_t total_ticks = 0;
    std::uint64_t final_checksum = 0;
    std::uint32_t checkpoint_interval = 0;  // 0 = no checkpoints
    std::vector<ReplayCheckpoint> checkpoints;
    CommandLog commands;

    void serialize(core::serialization::ByteWriter& writer) const;
    // Returns false on bad magic/version (corrupt reads still assert, per ByteReader policy).
    [[nodiscard]] bool deserialize(core::serialization::ByteReader& reader);
};

// Records a match as it is simulated. Owns the SimWorld so that nothing can
// advance the sim without the commands being logged.
class ReplayRecorder {
public:
    explicit ReplayRecorder(std::uint64_t seed, core::Scalar tick_duration_seconds = 1.0 / 60.0,
                            std::uint32_t checkpoint_interval = 60);

    // Logs `commands` against the current tick and advances one tick.
    void step(const std::vector<Command>& commands = {});

    // Applies `command` to the world IMMEDIATELY (so the caller gets the
    // spawned Entity handle back right away, which an engine integration
    // needs) and logs it against the current tick. Equivalent to passing it
    // to the next step(): ReplayPlayer applies logged commands first thing
    // in that same tick, and nothing runs between ticks. Returns spawned
    // entities (see apply_commands).
    std::vector<core::ecs::Entity> submit(const Command& command);

    [[nodiscard]] SimWorld& sim() noexcept { return sim_; }
    [[nodiscard]] const SimWorld& sim() const noexcept { return sim_; }
    [[nodiscard]] std::uint64_t current_tick() const noexcept { return sim_.ticks_simulated(); }

    // Finalizes (computes final checksum) and returns the recording. The
    // recorder may keep stepping afterwards; call finish() again for a newer one.
    [[nodiscard]] ReplayRecording finish() const;

private:
    SimWorld sim_;
    ReplayRecording rec_;
};

// Plays a recording back through a fresh SimWorld.
class ReplayPlayer {
public:
    explicit ReplayPlayer(const ReplayRecording& recording);

    // --- speed control -----------------------------------------------------
    void set_speed(double multiplier);  // clamped to [0, 64]; 0 behaves like pause
    [[nodiscard]] double speed() const noexcept { return speed_; }
    void pause() noexcept { paused_ = true; }
    void resume() noexcept { paused_ = false; }
    [[nodiscard]] bool paused() const noexcept { return paused_; }

    // Feed real elapsed seconds (e.g. engine frame delta). Runs however many
    // fixed ticks that corresponds to at the current speed; returns the number
    // run. Presentation-side only: never feeds time into the simulation.
    std::uint64_t advance(double real_seconds);

    // Runs exactly one tick (works while paused: frame-step). False if finished.
    bool step_tick();

    // Rewinds to tick 0 / fast-forwards to `tick` (clamped). Seeking
    // backwards re-simulates from the start (cheap enough for basic replay).
    void restart();
    void seek(std::uint64_t tick);

    [[nodiscard]] bool finished() const noexcept { return sim_->ticks_simulated() >= rec_.total_ticks; }
    [[nodiscard]] std::uint64_t current_tick() const noexcept { return sim_->ticks_simulated(); }
    [[nodiscard]] std::uint64_t total_ticks() const noexcept { return rec_.total_ticks; }
    [[nodiscard]] const SimWorld& sim() const noexcept { return *sim_; }

    // Presentation events produced by the ticks played since the last call
    // (attack started / projectile launched / died ...). Events are always
    // collected during playback; seek() and restart() discard pending ones.
    [[nodiscard]] std::vector<SimEvent> drain_events();

    // --- integrity ---------------------------------------------------------
    // True once any checkpoint hash was reached with a different state.
    [[nodiscard]] bool desynced() const noexcept { return desynced_; }
    // True iff playback is finished and the end state matches final_checksum.
    [[nodiscard]] bool final_checksum_matches() const;

private:
    void run_one_tick();

    const ReplayRecording& rec_;
    std::unique_ptr<SimWorld> sim_;
    double speed_ = 1.0;
    bool paused_ = false;
    double tick_accumulator_ = 0.0;  // in (fractional) ticks
    bool desynced_ = false;
};

}  // namespace thalassa::sim
