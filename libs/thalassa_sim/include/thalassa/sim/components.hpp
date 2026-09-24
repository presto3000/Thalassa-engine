#pragma once

#include "thalassa/core/math/vector.hpp"
#include "thalassa/core/types.hpp"

// Plain-data (trivially copyable) components for movement
// slice. Trivial copyability is load-bearing, not incidental: it's what
// lets ComponentPool<T>::serialize()/deserialize() (see component_pool.hpp)
// use a raw memcpy of the dense array instead of per-field logic, which is
// both faster and removes an entire category of serialization bugs (a
// field silently missing from hand-written (de)serialization code).

namespace thalassa::sim::components {

// World-space position. Deliberately just a position, no orientation —
// no facing-dependent gameplay yet (that arrives with
// abilities/attacks); adding Transform's rotation now would
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

}  // namespace thalassa::sim::components
