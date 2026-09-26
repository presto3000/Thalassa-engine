#pragma once

#include <cstdint>

#include "thalassa/core/math/vector.hpp"
#include "thalassa/core/types.hpp"

// Plain-data (trivially copyable) components for Milestone 1's movement
// slice. Trivial copyability is load-bearing, not incidental: it's what
// lets ComponentPool<T>::serialize()/deserialize() (see component_pool.hpp)
// use a raw memcpy of the dense array instead of per-field logic, which is
// both faster and removes an entire category of serialization bugs (a
// field silently missing from hand-written (de)serialization code).

namespace thalassa::sim::components {

// World-space position. Deliberately just a position, no orientation —
// Milestone 1 has no facing-dependent gameplay yet (that arrives with
// abilities/attacks in Milestone 2); adding Transform's rotation now would
// be speculative complexity.
struct Position {
    core::math::Vec3 value{0.0, 0.0, 0.0};
};

// Current per-tick velocity, in units/second. Written by the movement
// systems below (steering_system sets its direction/magnitude,
// integration_system consumes it); also readable by future systems that
// want current heading (e.g. presentation-layer interpolation, or a future
// facing/orientation system) without recomputing it from position deltas.
struct Velocity {
    core::math::Vec3 value{0.0, 0.0, 0.0};
};

// Presence of this component means "this entity wants to move toward
// `target`". steering_system removes it once the entity arrives (see
// movement.cpp), which is why it's a separate addable/removable component
// rather than an "is_active" bool living inside a permanent component —
// that keeps `each<Position, Destination>` queries automatically limited
// to only the entities currently traveling, no extra branch needed.
struct Destination {
    core::math::Vec3 target{0.0, 0.0, 0.0};
    core::Scalar speed = 5.0;            // units/second
    core::Scalar arrival_radius = 0.1;   // stop considering "arrived" within this distance
};

// --- Combat components (Milestone 2) ------------------------------------

// Which side an entity fights for. Deliberately a plain integer id, not an
// enum — the engine doesn't care how many teams exist or what they mean
// (2-team MOBA-style, N-team free-for-all, etc.); that's gameplay-content
// configuration, not something thalassa_sim hardcodes.
struct Team {
    std::int32_t id = 0;
};

// Which player (if any) owns/controls this entity. Separate from Team
// (multiple players can share a team) and currently just carried data —
// no system reads it yet in Milestone 2 — but it's the hook Milestone 3's
// "which client's input am I" and player-scoped commands attach to, so
// it's established now rather than retrofitted later.
struct Owner {
    core::PlayerId player{};
};

struct Health {
    core::Scalar current = 100.0;
    core::Scalar max = 100.0;
};

// A basic ranged "ability": attack_range/attack_damage/attack_cooldown_seconds
// are fixed configuration (set at spawn), cooldown_remaining_seconds is the
// only field combat_system mutates per tick. This is intentionally the
// simplest possible cooldown/cast framework per the milestone spec ("simple
// ability/skill framework") — one implicit ability per unit, auto-cast at
// the nearest enemy in range, no player-directed targeting or multiple
// abilities yet. See docs/milestone2_combat_design.md for why auto-cast
// rather than a full ability-selection system.
struct CombatStats {
    core::Scalar attack_damage = 10.0;
    core::Scalar attack_range = 15.0;
    core::Scalar attack_cooldown_seconds = 1.0;
    core::Scalar cooldown_remaining_seconds = 0.0;
};

// A deterministic ballistic projectile: straight-line travel (reuses
// Position+Velocity and integration_system — no separate projectile
// motion code needed), a fixed lifetime, and the damage/team it carries
// from the attack that spawned it. `owner_team` (not the owning Entity) is
// what collision checks against, so a projectile still deals damage
// correctly even if its firer died in the same tick it fired.
struct Projectile {
    core::Scalar damage = 0.0;
    std::int32_t owner_team = 0;
    core::Scalar ttl_seconds = 3.0;
};

}  // namespace thalassa::sim::components
