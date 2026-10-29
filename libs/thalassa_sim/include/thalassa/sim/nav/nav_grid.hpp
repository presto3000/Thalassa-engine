#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "thalassa/core/math/vector.hpp"
#include "thalassa/core/types.hpp"

// NavGrid: the static walkability map units navigate on. A uniform grid of square
// cells, each blocked or free. See docs/pathfinding_plan.md.
//
// Layout: cell (x, y) covers world XY [origin + x*cell_size, origin + (x+1)*cell_size),
// x to the right, y up (Thalassa coordinates, meters). Z is ignored. Everything outside
// the grid counts as blocked.
//
// Determinism: all queries are exact integer/IEEE operations (floor, +, *, compare), so
// every machine builds the same map from the same inputs. block_rect / block_circle mark
// a cell blocked iff its CENTER lies inside the shape.
//
// The map is match state: hash() identifies it (compare it between server and clients),
// serialize()/deserialize() move it through files and the network. deserialize() is safe
// on untrusted bytes.

namespace thalassa::sim::nav {

struct Cell {
    std::int32_t x = 0;
    constexpr bool operator==(const Cell&) const noexcept = default;
};

class NavGrid {
public:
    static constexpr std::int32_t kMaxDimension = 4096;  // cells per side

    // An all-free grid. width/height must be in [1, kMaxDimension], cell_size > 0 and finite
    // (violations are programming errors and assert).
    NavGrid(core::Scalar cell_size, core::Scalar origin_x, core::Scalar origin_y, std::int32_t width,
            std::int32_t height);

    [[nodiscard]] core::Scalar cell_size() const noexcept { return cell_size_; }
    [[nodiscard]] core::Scalar origin_x() const noexcept { return origin_x_; }
    [[nodiscard]] core::Scalar origin_y() const noexcept { return origin_y_; }
    [[nodiscard]] std::int32_t width() const noexcept { return width_; }
    [[nodiscard]] std::int32_t height() const noexcept { return height_; }
    [[nodiscard]] std::size_t cell_count() const noexcept { return blocked_.size(); }

    // Increases on every change; lets cached data (Pathfinder scratch) detect staleness.
    [[nodiscard]] std::uint64_t revision() const noexcept { return revision_; }

    [[nodiscard]] bool in_bounds(Cell c) const noexcept {
        return c.x >= 0 && c.y >= 0 && c.x < width_ && c.y < height_;
    }
    [[nodiscard]] bool blocked(Cell c) const noexcept { return !in_bounds(c) || blocked_[index_of(c)] != 0; }
    [[nodiscard]] bool free(Cell c) const noexcept { return !blocked(c); }

    // Out-of-range cells are ignored.
    void set_blocked(Cell c, bool is_blocked) noexcept;

    // Blocks (or frees) every cell whose center is inside the axis-aligned rectangle
    // [min_x, max_x] x [min_y, max_y] / inside the circle. Returns how many cells changed.
    std::size_t block_rect(core::Scalar min_x, core::Scalar min_y, core::Scalar max_x, core::Scalar max_y,
                           bool is_blocked = true);
    std::size_t block_circle(core::Scalar center_x, core::Scalar center_y, core::Scalar radius,
                             bool is_blocked = true);

    // The cell containing a world point, or nullopt if the point is outside the grid.
    [[nodiscard]] std::optional<Cell> to_cell(const core::math::Vec3& p) const noexcept;
    // Cell containing the point, extended past the border (may be out of bounds); for search start/goal snapping.
    [[nodiscard]] Cell to_cell_unclamped(const core::math::Vec3& p) const noexcept;
    [[nodiscard]] core::math::Vec3 cell_center(Cell c, core::Scalar z = 0.0) const noexcept;

    [[nodiscard]] std::size_t index_of(Cell c) const noexcept {
        return static_cast<std::size_t>(c.y) * static_cast<std::size_t>(width_) + static_cast<std::size_t>(c.x);
    }
    [[nodiscard]] Cell cell_of(std::size_t index) const noexcept {
        return Cell{static_cast<std::int32_t>(index % static_cast<std::size_t>(width_)),
                    static_cast<std::int32_t>(index / static_cast<std::size_t>(width_))};
    }

    // The free cell nearest (Euclidean, in cell units; ties -> lowest index) to `from`, searching
    // at most `max_radius_cells` away. nullopt if none.
    [[nodiscard]] std::optional<Cell> nearest_free_cell(Cell from, std::int32_t max_radius_cells) const noexcept;

    // FNV-1a over geometry and cells: equal maps hash equal.
    [[nodiscard]] std::uint64_t hash() const noexcept;

    [[nodiscard]] std::vector<std::byte> serialize() const;
    // Rejects truncated data, trailing bytes, bad magic/version, bad dimensions, non-finite or
    // non-positive cell size and cell bytes other than 0/1. Never asserts.
    [[nodiscard]] static std::optional<NavGrid> deserialize(std::span<const std::byte> bytes);

    bool operator==(const NavGrid& other) const noexcept;

private:
    core::Scalar cell_size_;
    core::Scalar origin_x_;
    core::Scalar origin_y_;
    std::int32_t width_;
    std::int32_t height_;
    std::vector<std::uint8_t> blocked_;  // 1 = blocked. (Packed bits later if memory matters.)
    std::uint64_t revision_ = 0;
};

}  // namespace thalassa::sim::nav
