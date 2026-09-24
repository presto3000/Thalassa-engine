#pragma once

#include "thalassa/core/ecs/world.hpp"
#include "thalassa/core/random.hpp"
#include "thalassa/core/types.hpp"
#include "thalassa/sim/spatial_index.hpp"
#include "thalassa/sim/systems/movement.hpp"

// SimWorld: the deterministic simulation state for one match/instance.
// Wraps thalassa_core's ECS World with the pieces every simulation needs
// regardless of gameplay content (a seeded Rng, the current tick counter,
// a spatial index) so thalassa_server can host N independent SimWorld
// instances per process (Milestone 3's "multiple concurrent game
// instances" requirement) without any shared mutable state between them.
//
// scope: movement only. tick() runs, in this fixed order:
//   1. steering_system   — Destination -> desired Velocity
//   2. integration_system — Velocity -> Position
//   3. spatial index rebuild — so this tick's queries (combat/AI) see this tick's post-movement positions
// Combat/ability systems will insert into this same fixed
// sequence, not run in parallel with it — see determinism rule 4 in
// docs/floating_point_policy.md.

namespace thalassa::sim {

    class SimWorld {
    public:
        explicit SimWorld(std::uint64_t rng_seed) : rng_(rng_seed) {}

        // Advances the simulation by exactly one fixed tick. Must remain a pure
        // function of (current world state, commands applied this tick) per
        // the determinism requirement — no wall-clock reads, no
        // non-deterministic container iteration, no floating-point outside
        // thalassa::core::math::strictfp.
        void tick(core::SimTick current_tick) {
            (void)current_tick;
            const core::Scalar dt = tick_duration_seconds_;

            systems::steering_system(ecs_world_, dt);
            systems::integration_system(ecs_world_, dt);
            spatial_index_.rebuild(ecs_world_);

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
