#include "thalassa/sim/nav/pathfinder.hpp"

#include <algorithm>
#include <queue>

namespace thalassa::sim::nav {

namespace {

constexpr std::int32_t kStraight = 10;
constexpr std::int32_t kDiagonal = 14;

struct Step {
    std::int32_t dx, dy, cost;
};
// Fixed expansion order: N E S W, then the diagonals. Never reorder: it is part of the tie-break.
constexpr Step kSteps[8] = {{0, -1, kStraight}, {1, 0, kStraight},  {0, 1, kStraight},   {-1, 0, kStraight},
                            {1, -1, kDiagonal}, {1, 1, kDiagonal}, {-1, 1, kDiagonal}, {-1, -1, kDiagonal}};

// Octile distance in tenths of a cell: consistent for this cost model.
std::int32_t heuristic(Cell a, Cell b) {
    const std::int32_t dx = a.x > b.x ? a.x - b.x : b.x - a.x;
    const std::int32_t dy = a.y > b.y ? a.y - b.y : b.y - a.y;
    const std::int32_t lo = dx < dy ? dx : dy;
    return kStraight * (dx + dy) - (2 * kStraight - kDiagonal) * lo;
}

struct OpenEntry {
    std::int32_t f;
    std::int32_t h;
    std::int32_t index;
    std::int32_t g;
};
// std::priority_queue pops the "largest": make that the entry with the smallest (f, h, index, g).
struct LowerPriority {
    bool operator()(const OpenEntry& a, const OpenEntry& b) const {
        if (a.f != b.f) return a.f > b.f;
        if (a.h != b.h) return a.h > b.h;
        if (a.index != b.index) return a.index > b.index;
        return a.g > b.g;
    }
};

}  // namespace

void Pathfinder::prepare() {
    if (nodes_.size() != grid_->cell_count()) {
        nodes_.assign(grid_->cell_count(), Node{});
        generation_ = 0;
    }
    prepared_revision_ = grid_->revision();
    prepared_cells_ = grid_->cell_count();
}

PathResult Pathfinder::find_path(const core::math::Vec3& start, const core::math::Vec3& goal,
                                 const PathOptions& options) {
    PathResult result;
    const NavGrid& grid = *grid_;
    prepare();

    // New search: bump the generation instead of clearing the arrays.
    if (++generation_ == 0) {  // wrapped: really clear once every 4 billion searches
        for (Node& n : nodes_) n = Node{};
        generation_ = 1;
    }

    const Cell raw_start = grid.to_cell_unclamped(start);
    const Cell raw_goal = grid.to_cell_unclamped(goal);
    const std::optional<Cell> start_cell = grid.nearest_free_cell(raw_start, options.snap_radius_cells);
    const std::optional<Cell> goal_cell = grid.nearest_free_cell(raw_goal, options.snap_radius_cells);
    if (!start_cell || !goal_cell) return result;  // NoPath

    const bool start_snapped = !(*start_cell == raw_start);
    const bool goal_snapped = !(*goal_cell == raw_goal);

    auto touch = [&](std::size_t idx) -> Node& {
        Node& n = nodes_[idx];
        if (n.stamp != generation_) {
            n = Node{};
            n.g = 0x3fffffff;
            n.stamp = generation_;
        }
        return n;
    };

    const auto start_idx = static_cast<std::int32_t>(grid.index_of(*start_cell));
    const auto goal_idx = static_cast<std::int32_t>(grid.index_of(*goal_cell));

    std::priority_queue<OpenEntry, std::vector<OpenEntry>, LowerPriority> open;
    {
        Node& s = touch(static_cast<std::size_t>(start_idx));
        s.g = 0;
        const std::int32_t h = heuristic(*start_cell, *goal_cell);
        open.push(OpenEntry{h, h, start_idx, 0});
    }

    // Best cell seen so far by distance to the goal, for Partial results.
    std::int32_t best_idx = start_idx;
    std::int32_t best_h = heuristic(*start_cell, *goal_cell);
    std::int32_t best_g = 0;

    bool reached = false;
    while (!open.empty()) {
        const OpenEntry cur = open.top();
        open.pop();
        Node& node = nodes_[static_cast<std::size_t>(cur.index)];
        if (node.closed || cur.g != node.g) continue;  // stale entry
        node.closed = true;

        if (cur.index == goal_idx) {
            reached = true;
            best_idx = cur.index;
            best_g = cur.g;
            break;
        }
        if (result.expansions >= options.max_expansions) break;
        ++result.expansions;

        if (cur.h < best_h || (cur.h == best_h && cur.g < best_g)) {
            best_h = cur.h;
            best_idx = cur.index;
            best_g = cur.g;
        }

        const Cell c = grid.cell_of(static_cast<std::size_t>(cur.index));
        for (const Step& s : kSteps) {
            const Cell n{c.x + s.dx, c.y + s.dy};
            if (grid.blocked(n)) continue;
            if (s.dx != 0 && s.dy != 0 && (grid.blocked(Cell{c.x + s.dx, c.y}) || grid.blocked(Cell{c.x, c.y + s.dy}))) {
                continue;  // no corner cutting
            }
            const auto nidx = static_cast<std::int32_t>(grid.index_of(n));
            Node& nn = touch(static_cast<std::size_t>(nidx));
            if (nn.closed) continue;
            const std::int32_t g = cur.g + s.cost;
            if (g >= nn.g) continue;
            nn.g = g;
            nn.parent = cur.index;
            const std::int32_t h = heuristic(n, *goal_cell);
            open.push(OpenEntry{g + h, h, nidx, g});
        }
    }

    // Nothing closer than where we stand (and not already there): no useful movement.
    if (!reached && best_idx == start_idx && !(*start_cell == *goal_cell)) return result;

    result.status = reached ? PathStatus::Found : PathStatus::Partial;
    result.cost = nodes_[static_cast<std::size_t>(best_idx)].g;

    // Walk parents back to the start, then reverse.
    std::vector<Cell> cells;
    for (std::int32_t i = best_idx; i != -1; i = nodes_[static_cast<std::size_t>(i)].parent) {
        cells.push_back(grid.cell_of(static_cast<std::size_t>(i)));
    }
    std::reverse(cells.begin(), cells.end());

    // The start cell itself is where the unit already is, unless we had to move it out of an obstacle.
    const std::size_t first = start_snapped ? 0 : 1;
    for (std::size_t i = first; i < cells.size(); ++i) {
        result.waypoints.push_back(grid.cell_center(cells[i], start.z));
    }
    // Land exactly on the requested point when its own cell was the goal.
    if (reached && !goal_snapped) {
        const core::math::Vec3 exact{goal.x, goal.y, goal.z};
        if (result.waypoints.empty()) {
            result.waypoints.push_back(exact);  // start and goal share a cell
        } else {
            result.waypoints.back() = exact;
        }
    }
    return result;
}

}  // namespace thalassa::sim::nav
