#pragma once

#include <vector>

#include "thalassa/core/ecs/world.hpp"
#include "thalassa/core/random.hpp"
#include "thalassa/core/types.hpp"
#include "thalassa/sim/commands.hpp"
#include "thalassa/sim/spatial_index.hpp"
#include "thalassa/sim/systems/combat.hpp"
#include "thalassa/sim/systems/movement.hpp"

// SimWorld: the deterministic simulation state for one match/instance.
// Wraps thalassa_core's ECS World with the pieces every simulation needs
// regardless of gameplay content (a seeded Rng, the current tick counter,
// a spatial index) so thalassa_server can host N independent SimWorld
// instances per process (Milestone 3's "multiple concurrent game
// instances" requirement) without any shared mutable state between them.
//
// tick() runs, in this fixed order every tick:
//   1. apply_commands   — Command buffer -> World mutations (spawns, move orders)
//   2. steering_system   — Destination -> desired Velocity
//   3. integration_system — Velocity -> Position (moves units AND projectiles)
//   4. spatial index rebuild — so the rest of this tick's queries see
//      this tick's post-movement positions
//   5. combat_system     — cooldowns tick down, in-range attacks fire (may spawn projectiles)
//   6. projectile_collision_system — moving projectiles check for hits, apply damage, expire
//   7. death_system      — remove anything whose Health reached zero
// Every later milestone's systems insert into this same fixed sequence,
// never run in parallel with it — see determinism rule 4 in
// docs/floating_point_policy.md.

namespace thalassa::sim {

class SimWorld {
public:
    explicit SimWorld(std::uint64_t rng_seed) : rng_(rng_seed) {}

    // Convenience overload for callers with no commands this tick, movement-only tests/sandbox) — equivalent to
    // tick(current_tick, {}).
    void tick(core::SimTick current_tick) { tick(current_tick, {}); }

    // Advances the simulation by exactly one fixed tick, applying
    // `commands` first. Must remain a pure function of (current world
    // state, commands applied this tick) per the determinism requirement —
    // no wall-clock reads, no non-deterministic container iteration, no
    // floating-point outside thalassa::core::math::strictfp.
    void tick(core::SimTick current_tick, const std::vector<Command>& commands) {
        (void)current_tick;
        const core::Scalar dt = tick_duration_seconds_;

        (void)apply_commands(ecs_world_, commands);

        systems::steering_system(ecs_world_, dt);
        systems::integration_system(ecs_world_, dt);
        spatial_index_.rebuild(ecs_world_);

        systems::combat_system(ecs_world_, spatial_index_, dt);
        systems::projectile_collision_system(ecs_world_, spatial_index_, dt);
        systems::death_system(ecs_world_);

        ++ticks_simulated_;
    }

    [[nodiscard]] core::ecs::World& world() noexcept { return ecs_world_; }
    [[nodiscard]] const core::ecs::World& world() const noexcept { return ecs_world_; }
    [[nodiscard]] core::Rng& rng() noexcept { return rng_; }
    [[nodiscard]] const core::Rng& rng() const noexcept { return rng_; }
    [[nodiscard]] std::uint64_t ticks_simulated() const noexcept { return ticks_simulated_; }
    [[nodiscard]] const SpatialIndex& spatial_index() const noexcept { return spatial_index_; }

    // --- Snapshot-restore support (see thalassa/sim/replay.hpp) -----------
    //
    // Not intended for general gameplay use — these exist so
    // load_world() can restore tick-counter/spatial-index state that
    // tick() would otherwise be the only way to advance. Kept as narrow,
    // clearly-named setters rather than making the underlying fields
    // public, so it's obvious at every call site that this is a
    // load-path-only operation.
    void set_ticks_simulated(std::uint64_t ticks) noexcept { ticks_simulated_ = ticks; }
    void rebuild_spatial_index() { spatial_index_.rebuild(ecs_world_); }

    // Fixed per-tick simulated time. Must match whatever FixedTimestep the
    // caller (HeadlessApp) is driven by — see set_tick_duration_seconds().
    // Defaults to 1/60s so SimWorld is usable standalone (tests, sandbox)
    // without a HeadlessApp wiring it up.
    void set_tick_duration_seconds(core::Scalar seconds) noexcept { tick_duration_seconds_ = seconds; }
    [[nodiscard]] core::Scalar tick_duration_seconds() const noexcept { return tick_duration_seconds_; }

private:
    core::ecs::World ecs_world_;
    core::Rng rng_;
    SpatialIndex spatial_index_;
    std::uint64_t ticks_simulated_ = 0;
    core::Scalar tick_duration_seconds_ = 1.0 / 60.0;
};

}  // namespace thalassa::sim
