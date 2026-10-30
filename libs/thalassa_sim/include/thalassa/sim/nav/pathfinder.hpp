#pragma once

#include <cstdint>
#include <vector>

#include "thalassa/core/math/vector.hpp"
#include "thalassa/sim/nav/nav_grid.hpp"

// Pathfinder: deterministic A* over a NavGrid. See docs/pathfinding_plan.md.
//
//   Pathfinder finder(grid);
//   PathResult r = finder.find_path(unit_position, destination);
//
// - 8-way movement, cost 10 straight / 14 diagonal; a diagonal step needs both side
//   cells free (no cutting corners).
// - Search is pure integer arithmetic with a total tie-break order (f, h, cell index, g),
//   so the same query on the same map returns the same path on every machine.
// - A blocked start or goal is moved to the nearest free cell (within snap_radius_cells).
// - If the goal cannot be reached (walled in, or the budget ran out) the result is
//   Partial: a path to the explored cell closest to the goal, the usual RTS "get as close
//   as you can". NoPath only when no useful movement exists at all.
// - Reuses its scratch memory between queries; one Pathfinder per thread. It keeps a
//   pointer to the grid, which must outlive it; changing the grid between queries is fine.
//
// Raw waypoints are cell centers (the last one is the exact goal point when the goal's own
// cell is free), start excluded unless the start had to be snapped. With options.smooth
// (the default) they are then pulled straight (see smooth_waypoints).
//
// Unit size: search an inflated copy of the map, `grid.inflated(unit_radius)`, one per size
// class; a point on the inflated map is a circle on the real one.

namespace thalassa::sim::nav {

enum class PathStatus : std::uint8_t {
    Found,    // reached the goal (or the free cell nearest to it)
    Partial,  // goal unreachable / budget exhausted: path to the closest cell reached
    NoPath,   // no movement possible (start enclosed, outside the map, or nothing closer than the start)
};

struct PathOptions {
    std::uint32_t max_expansions = 200000;   // cells the search may expand before giving up (Partial)
    std::int32_t snap_radius_cells = 16;     // how far a blocked start/goal may move to find a free cell
    bool smooth = true;                      // shortcut the grid path into straight segments (string pulling)
    // Debug/tooling output (off by default, costs nothing when off):
    bool keep_raw = false;                   // also return the unsmoothed grid path in PathResult::raw_waypoints
    bool record_explored = false;            // also return every expanded cell, in expansion order
};

struct PathResult {
    PathStatus status = PathStatus::NoPath;
    std::vector<core::math::Vec3> waypoints;      // what units should follow (smoothed unless options.smooth == false)
    std::vector<core::math::Vec3> raw_waypoints;  // the grid path before smoothing (only if options.keep_raw)
    std::vector<Cell> explored;                   // expanded cells in order (only if options.record_explored)
    std::uint32_t expansions = 0;     // search effort, for tests/profiling
    std::int32_t cost = 0;            // grid path cost in tenths of a cell (10 per straight step); 0 if none
};

// String pulling: from `start`, repeatedly jump to the furthest waypoint that is in clear
// line of sight (NavGrid::segment_clear), so the grid zig-zag becomes straight legs that
// bend only around obstacles. The last waypoint is always kept (the destination). Where no
// shortcut exists the next original waypoint is kept (a legal grid step), so the result is
// never worse than the input: never longer, never more points, never crossing a blocked cell
// that the input did not already cross.
[[nodiscard]] std::vector<core::math::Vec3> smooth_waypoints(const NavGrid& grid, const core::math::Vec3& start,
                                                             const std::vector<core::math::Vec3>& waypoints);

class Pathfinder {
public:
    explicit Pathfinder(const NavGrid& grid) : grid_(&grid) {}

    [[nodiscard]] PathResult find_path(const core::math::Vec3& start, const core::math::Vec3& goal,
                                       const PathOptions& options = {});

private:
    struct Node {
        std::int32_t g = 0;
        std::int32_t parent = -1;
        std::uint32_t stamp = 0;   // == generation_ if touched by the current search
        bool closed = false;
    };

    void prepare();

    const NavGrid* grid_;
    std::vector<Node> nodes_;
    std::uint32_t generation_ = 0;
    std::uint64_t prepared_revision_ = ~0ULL;
    std::size_t prepared_cells_ = 0;
};

}  // namespace thalassa::sim::nav
