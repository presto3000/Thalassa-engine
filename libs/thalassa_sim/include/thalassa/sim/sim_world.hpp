#pragma once

#include "thalassa/core/ecs/world.hpp"
#include "thalassa/core/random.hpp"
#include "thalassa/core/types.hpp"

// SimWorld: the deterministic simulation state for one match/instance.
// Wraps thalassa_core's ECS World with the pieces every simulation needs
// regardless of gameplay content (a seeded Rng, the current tick counter)
// so thalassa_server can host N independent SimWorld instances per process
// without any shared mutable state between them.
//
// scope: an empty world that advances its tick counter and
// nothing else. To add: components (Transform, Velocity,
// Destination) and systems (movement, spatial queries) that operate on
// `ecs_world` inside `tick()`.

namespace thalassa::sim {

class SimWorld {
public:
    explicit SimWorld(std::uint64_t rng_seed) : rng_(rng_seed) {}

    // Advances the simulation by exactly one fixed tick. Must remain a pure
    // function of (current world state, commands applied this tick) per
    // the determinism requirement — no wall-clock reads, no
    // non-deterministic container iteration, no floating-point outside
    // thalassa::core::math::strictfp. TODO: populate this with
    // actual systems (movement, combat, etc); It advances the
    // tick counter to prove the loop wiring is correct end-to-end.
    void tick(core::SimTick current_tick) {
        (void)current_tick;
        // Systems will run here, in a fixed, explicitly-ordered sequence
        // (never a container-iteration-order-dependent sequence), e.g.:
        //   movement_system(ecs_world, dt);
        //   combat_system(ecs_world, dt);
        ++ticks_simulated_;
    }

    [[nodiscard]] core::ecs::World& world() noexcept { return ecs_world_; }
    [[nodiscard]] const core::ecs::World& world() const noexcept { return ecs_world_; }
    [[nodiscard]] core::Rng& rng() noexcept { return rng_; }
    [[nodiscard]] std::uint64_t ticks_simulated() const noexcept { return ticks_simulated_; }

private:
    core::ecs::World ecs_world_;
    core::Rng rng_;
    std::uint64_t ticks_simulated_ = 0;
};

}  // namespace thalassa::sim
