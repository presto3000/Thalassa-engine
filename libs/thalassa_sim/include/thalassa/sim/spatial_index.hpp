#pragma once

#include <cstddef>
#include <vector>

#include "thalassa/core/ecs/world.hpp"
#include "thalassa/core/spatial/uniform_grid.hpp"
#include "thalassa/sim/components.hpp"

// SpatialIndex: "basic spatial structure ... for queries"
// deliverable, wiring thalassa_core's generic UniformGrid to this project's
// Position component. Rebuilt from scratch once per tick (see
// SimWorld::tick) by iterating the Position pool in its deterministic
// insertion order — see uniform_grid.hpp's header comment for why a full
// per-tick rebuild is the right tradeoff for target entity
// counts, and why query results are deterministic regardless of the
// underlying hash map's iteration order.

namespace thalassa::sim {

class SpatialIndex {
public:
    explicit SpatialIndex(core::Scalar cell_size = 5.0) : grid_(cell_size) {}

    // Clears and re-inserts every entity that currently has a Position
    // component, in Position-pool insertion order. O(n) — call once per
    // tick, not per query.
    void rebuild(const core::ecs::World& world) {
        grid_.clear();
        if (const auto* positions = world.find_pool<components::Position>()) {
            const auto& entities = positions->entities();
            const auto& values = positions->values();
            for (std::size_t i = 0; i < entities.size(); ++i) {
                grid_.insert(entities[i], values[i].value);
            }
        }
    }

    void query_radius(const core::math::Vec3& center, core::Scalar radius,
                       std::vector<core::ecs::Entity>& out) const {
        grid_.query_radius(center, radius, out);
    }

    void query_aabb(const core::math::Vec3& min, const core::math::Vec3& max,
                     std::vector<core::ecs::Entity>& out) const {
        grid_.query_aabb(min, max, out);
    }

    [[nodiscard]] std::size_t size() const noexcept { return grid_.size(); }

private:
    core::spatial::UniformGrid<core::ecs::Entity> grid_;
};

}  // namespace thalassa::sim
