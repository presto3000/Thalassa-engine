#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "thalassa/core/math/vector.hpp"
#include "thalassa/core/types.hpp"

// UniformGrid<Handle>: a generic uniform-grid broad-phase spatial index.
// Templated on a small, trivially-copyable "handle" type (typically
// thalassa::core::ecs::Entity, but kept decoupled from ECS specifically so
// this is independently usable/testable — see docs/ecs_decision.md's
// "exactly the surface we need" philosophy applied to a second module).
//
// Determinism: entries are stored per-cell in *insertion order*
// (std::vector, not a set), and every query traverses candidate cells by
// explicitly walking integer cell coordinates in ascending (cx, then cy)
// order — never by iterating the underlying std::unordered_map directly.
// That means query result order is fully deterministic (a function of
// insertion order and the query parameters only) regardless of
// std::unordered_map's unspecified bucket/iteration order. This is the
// same "drive traversal ourselves, never trust container iteration order"
// principle as thalassa_core::ecs::World (see determinism rule 3 in
// docs/floating_point_policy.md).
//
// Only the XY plane is bucketed (Z is stored per-entry and used for exact
// distance checks, but doesn't affect which cell an entry lives in) — this
// matches a top-down RTS/MOBA ground plane; revisit if verticality
// (cliffs/flying units needing 3D broad-phase) becomes gameplay-relevant.
//
// scope: full clear()+re-insert per query cycle (typically
// once per sim tick, see thalassa::sim::SimWorld). This is O(entity count)
// per tick, which is intentionally simple and correct; an incremental
// update API (move an entry without a full rebuild) is a likely
// optimization once profiling says a full rebuild is the bottleneck at
// higher entity counts than targets (5000-10000).

namespace thalassa::core::spatial {

template <typename Handle>
class UniformGrid {
public:
    struct Entry {
        Handle handle;
        math::Vec3 position;
    };

    explicit UniformGrid(Scalar cell_size) noexcept : cell_size_(cell_size) {}

    void clear() noexcept {
        cells_.clear();
        count_ = 0;
    }

    void insert(Handle handle, const math::Vec3& position) {
        const CellKey key = cell_key(position);
        cells_[key].push_back(Entry{handle, position});
        ++count_;
    }

    [[nodiscard]] std::size_t size() const noexcept { return count_; }

    // Appends every entry within `radius` of `center` (inclusive, using
    // squared-distance comparison — no sqrt needed) to `out`, in
    // deterministic cell-then-insertion order. Does not clear `out` first,
    // so callers can accumulate across calls if useful.
    void query_radius(const math::Vec3& center, Scalar radius, std::vector<Handle>& out) const {
        const Scalar radius_sq = radius * radius;
        const CellCoord min_cell = cell_coord(center.x - radius, center.y - radius);
        const CellCoord max_cell = cell_coord(center.x + radius, center.y + radius);

        for (std::int64_t cy = min_cell.cy; cy <= max_cell.cy; ++cy) {
            for (std::int64_t cx = min_cell.cx; cx <= max_cell.cx; ++cx) {
                const auto it = cells_.find(pack(cx, cy));
                if (it == cells_.end()) {
                    continue;
                }
                for (const Entry& entry : it->second) {
                    const math::Vec3 delta = entry.position - center;
                    if (delta.length_sq() <= radius_sq) {
                        out.push_back(entry.handle);
                    }
                }
            }
        }
    }

    // Appends every entry whose position falls within the axis-aligned box
    // [min, max] (inclusive) to `out`, in deterministic cell-then-insertion
    // order.
    void query_aabb(const math::Vec3& min, const math::Vec3& max, std::vector<Handle>& out) const {
        const CellCoord min_cell = cell_coord(min.x, min.y);
        const CellCoord max_cell = cell_coord(max.x, max.y);

        for (std::int64_t cy = min_cell.cy; cy <= max_cell.cy; ++cy) {
            for (std::int64_t cx = min_cell.cx; cx <= max_cell.cx; ++cx) {
                const auto it = cells_.find(pack(cx, cy));
                if (it == cells_.end()) {
                    continue;
                }
                for (const Entry& entry : it->second) {
                    if (entry.position.x >= min.x && entry.position.x <= max.x &&
                        entry.position.y >= min.y && entry.position.y <= max.y) {
                        out.push_back(entry.handle);
                    }
                }
            }
        }
    }

    [[nodiscard]] Scalar cell_size() const noexcept { return cell_size_; }

private:
    using CellKey = std::int64_t;
    struct CellCoord {
        std::int64_t cx;
        std::int64_t cy;
    };

    [[nodiscard]] CellCoord cell_coord(Scalar x, Scalar y) const noexcept {
        // floor-division toward negative infinity so negative coordinates
        // bucket correctly (game worlds are frequently centered on origin).
        auto floor_div = [this](Scalar v) noexcept -> std::int64_t {
            const Scalar scaled = v / cell_size_;
            const auto truncated = static_cast<std::int64_t>(scaled);
            return (scaled < static_cast<Scalar>(truncated)) ? truncated - 1 : truncated;
        };
        return CellCoord{floor_div(x), floor_div(y)};
    }

    [[nodiscard]] CellKey cell_key(const math::Vec3& position) const noexcept {
        const CellCoord c = cell_coord(position.x, position.y);
        return pack(c.cx, c.cy);
    }

    // Packs two 32-bit-range coordinates into a single 64-bit key. Game
    // worlds are assumed to fit within +/-  ~2^31 grid cells in each axis,
    // which at even a 0.1-unit cell size covers a ~200-million-unit map —
    // far beyond any planned map size.
    [[nodiscard]] static CellKey pack(std::int64_t cx, std::int64_t cy) noexcept {
        return (cx << 32) ^ (cy & 0xFFFFFFFFLL);
    }

    Scalar cell_size_;
    std::unordered_map<CellKey, std::vector<Entry>> cells_;
    std::size_t count_ = 0;
};

}  // namespace thalassa::core::spatial
