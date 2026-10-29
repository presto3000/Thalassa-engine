#include <cmath>
#include <cstring>
#include <cstdint>
#include <queue>
#include <set>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "thalassa/core/random.hpp"
#include "thalassa/sim/nav/nav_grid.hpp"
#include "thalassa/sim/nav/pathfinder.hpp"

using namespace thalassa;
using core::math::Vec3;
using sim::nav::Cell;
using sim::nav::NavGrid;
using sim::nav::PathOptions;
using sim::nav::PathResult;
using sim::nav::PathStatus;
using sim::nav::Pathfinder;

namespace {

// Grid with origin (0,0), 1 m cells: cell (x,y) has its center at (x+0.5, y+0.5).
NavGrid open_grid(int w, int h) { return NavGrid(1.0, 0.0, 0.0, w, h); }

Vec3 at(double x, double y) { return Vec3{x, y, 0.0}; }

// Independent reference: Dijkstra with the same move rules; returns best cost in tenths, -1 if unreachable.
int reference_cost(const NavGrid& g, Cell from, Cell to) {
    const int w = g.width(), h = g.height();
    std::vector<int> dist(static_cast<std::size_t>(w * h), 1 << 30);
    using Item = std::pair<int, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<>> pq;
    dist[static_cast<std::size_t>(from.y * w + from.x)] = 0;
    pq.push({0, from.y * w + from.x});
    while (!pq.empty()) {
        const auto [d, idx] = pq.top();
        pq.pop();
        if (d > dist[static_cast<std::size_t>(idx)]) continue;
        const int cx = idx % w, cy = idx / w;
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) continue;
                const Cell n{cx + dx, cy + dy};
                if (g.blocked(n)) continue;
                if (dx != 0 && dy != 0 && (g.blocked(Cell{cx + dx, cy}) || g.blocked(Cell{cx, cy + dy}))) continue;
                const int nd = d + (dx != 0 && dy != 0 ? 14 : 10);
                const auto ni = static_cast<std::size_t>(n.y * w + n.x);
                if (nd < dist[ni]) {
                    dist[ni] = nd;
                    pq.push({nd, n.y * w + n.x});
                }
            }
        }
    }
    const int d = dist[static_cast<std::size_t>(to.y * w + to.x)];
    return d == (1 << 30) ? -1 : d;
}

// Every consecutive waypoint pair must be a legal single step (straight or diagonal without corner cutting).
void require_legal_steps(const NavGrid& g, const Vec3& start, const PathResult& r) {
    Cell prev = *g.to_cell(start);
    for (const Vec3& wp : r.waypoints) {
        const auto cell = g.to_cell(wp);
        REQUIRE(cell.has_value());
        REQUIRE(g.free(*cell));
        const int dx = cell->x - prev.x, dy = cell->y - prev.y;
        REQUIRE(std::abs(dx) <= 1);
        REQUIRE(std::abs(dy) <= 1);
        if (dx != 0 && dy != 0) {
            REQUIRE(g.free(Cell{prev.x + dx, prev.y}));
            REQUIRE(g.free(Cell{prev.x, prev.y + dy}));
        }
        prev = *cell;
    }
}

}  // namespace

TEST_CASE("nav grid: coordinates, bounds and blocking", "[nav]") {
    NavGrid g(2.0, -10.0, -4.0, 10, 5);  // covers x in [-10, 10), y in [-4, 6)
    CHECK(g.cell_count() == 50);
    CHECK(*g.to_cell(at(-10.0, -4.0)) == Cell{0, 0});
    CHECK(*g.to_cell(at(-8.01, -2.01)) == Cell{0, 0});
    CHECK(*g.to_cell(at(-8.0, -2.0)) == Cell{1, 1});
    CHECK(*g.to_cell(at(9.99, 5.99)) == Cell{9, 4});
    CHECK_FALSE(g.to_cell(at(10.0, 0.0)).has_value());
    CHECK_FALSE(g.to_cell(at(-10.01, 0.0)).has_value());
    CHECK(g.cell_center(Cell{0, 0}) == at(-9.0, -3.0));
    CHECK(g.to_cell_unclamped(at(-12.5, 0.0)).x == -2);  // floor, not truncation
    CHECK(g.blocked(Cell{-1, 0}));                       // outside = blocked
    CHECK(g.blocked(Cell{10, 0}));

    CHECK(g.free(Cell{3, 3}));
    const auto rev = g.revision();
    g.set_blocked(Cell{3, 3}, true);
    CHECK(g.blocked(Cell{3, 3}));
    CHECK(g.revision() > rev);
    g.set_blocked(Cell{3, 3}, true);  // no change: revision stays
    CHECK(g.revision() == rev + 1);
    g.set_blocked(Cell{99, 99}, true);  // ignored

    // Cells are blocked by CENTER: a 2x2 m square over four whole cells.
    NavGrid h = open_grid(10, 10);
    CHECK(h.block_rect(2.0, 2.0, 4.0, 4.0) == 4);  // centers 2.5 and 3.5 in both axes
    CHECK(h.blocked(Cell{2, 2}));
    CHECK(h.blocked(Cell{3, 3}));
    CHECK(h.free(Cell{4, 4}));
    CHECK(h.block_circle(7.5, 7.5, 1.0) == 5);  // the center cell and its four neighbours
    CHECK(h.block_rect(2.0, 2.0, 4.0, 4.0, false) == 4);  // freeing works too
    CHECK(h.free(Cell{2, 2}));
}

TEST_CASE("nav grid: nearest free cell", "[nav]") {
    NavGrid g = open_grid(9, 9);
    (void)g.block_rect(3.0, 3.0, 6.0, 6.0);  // blocks cells 3..5 in x and y (centers 3.5, 4.5, 5.5)
    REQUIRE(g.blocked(Cell{4, 4}));
    CHECK(g.nearest_free_cell(Cell{0, 0}, 5) == Cell{0, 0});  // already free
    const auto n = g.nearest_free_cell(Cell{4, 4}, 5);
    REQUIRE(n.has_value());
    CHECK(g.free(*n));
    CHECK((n->x == 2 || n->x == 6 || n->y == 2 || n->y == 6));  // just outside the block, two cells away
    CHECK_FALSE(g.nearest_free_cell(Cell{4, 4}, 1).has_value());  // radius too small
    // Deterministic: asking twice gives the same answer.
    CHECK(g.nearest_free_cell(Cell{4, 4}, 5) == n);
    // Outside the map still finds the border.
    CHECK(g.nearest_free_cell(Cell{-3, 4}, 6).has_value());
}

TEST_CASE("nav grid: hash, equality and serialization", "[nav]") {
    NavGrid a = open_grid(16, 12);
    (void)a.block_circle(8.0, 6.0, 3.0);
    NavGrid b = open_grid(16, 12);
    CHECK(a.hash() != b.hash());
    (void)b.block_circle(8.0, 6.0, 3.0);
    CHECK(a.hash() == b.hash());
    CHECK(a == b);
    b.set_blocked(Cell{0, 0}, true);
    CHECK(a.hash() != b.hash());
    CHECK_FALSE(a == b);

    const auto bytes = a.serialize();
    const auto back = NavGrid::deserialize(bytes);
    REQUIRE(back.has_value());
    CHECK(*back == a);
    CHECK(back->hash() == a.hash());
}

TEST_CASE("nav grid: deserialize rejects malformed input without crashing", "[nav]") {
    NavGrid g = open_grid(8, 8);
    (void)g.block_rect(2.0, 2.0, 3.0, 3.0);
    const auto good = g.serialize();
    REQUIRE(NavGrid::deserialize(good).has_value());

    // Every truncation and one trailing byte.
    for (std::size_t n = 0; n < good.size(); ++n) {
        CHECK_FALSE(NavGrid::deserialize(std::span<const std::byte>(good.data(), n)).has_value());
    }
    auto extra = good;
    extra.push_back(std::byte{0});
    CHECK_FALSE(NavGrid::deserialize(extra).has_value());

    auto bad_magic = good;
    bad_magic[0] = std::byte{0x00};
    CHECK_FALSE(NavGrid::deserialize(bad_magic).has_value());
    auto bad_cell = good;
    bad_cell.back() = std::byte{7};
    CHECK_FALSE(NavGrid::deserialize(bad_cell).has_value());

    // Absurd dimensions / cell size in the header.
    auto huge = good;
    const std::int32_t big = 1 << 30;
    std::memcpy(huge.data() + 32, &big, sizeof(big));  // width
    CHECK_FALSE(NavGrid::deserialize(huge).has_value());
    auto nan_size = good;
    const double nan = std::nan("");
    std::memcpy(nan_size.data() + 8, &nan, sizeof(nan));
    CHECK_FALSE(NavGrid::deserialize(nan_size).has_value());
    auto zero_size = good;
    const double zero = 0.0;
    std::memcpy(zero_size.data() + 8, &zero, sizeof(zero));
    CHECK_FALSE(NavGrid::deserialize(zero_size).has_value());
    CHECK_FALSE(NavGrid::deserialize({}).has_value());
}

TEST_CASE("pathfinder: open field, straight and diagonal", "[nav][path]") {
    const NavGrid g = open_grid(20, 20);
    Pathfinder f(g);

    const PathResult straight = f.find_path(at(1.5, 1.5), at(8.5, 1.5));
    REQUIRE(straight.status == PathStatus::Found);
    CHECK(straight.cost == 70);
    REQUIRE(straight.waypoints.size() == 7);
    CHECK(straight.waypoints.back() == at(8.5, 1.5));  // exact goal point
    require_legal_steps(g, at(1.5, 1.5), straight);

    const PathResult diag = f.find_path(at(1.5, 1.5), at(6.2, 6.9));
    REQUIRE(diag.status == PathStatus::Found);
    CHECK(diag.cost == 5 * 14);
    CHECK(diag.waypoints.back() == at(6.2, 6.9));

    // Start and goal in the same cell: one waypoint, the goal itself.
    const PathResult same = f.find_path(at(3.1, 3.1), at(3.9, 3.4));
    REQUIRE(same.status == PathStatus::Found);
    REQUIRE(same.waypoints.size() == 1);
    CHECK(same.waypoints[0] == at(3.9, 3.4));
    CHECK(same.cost == 0);
}

TEST_CASE("pathfinder: walks around a wall through the gap", "[nav][path]") {
    NavGrid g = open_grid(20, 20);
    (void)g.block_rect(10.0, 0.0, 11.0, 20.0);  // vertical wall at x = 10 (cell column 10)
    g.set_blocked(Cell{10, 15}, false);          // one gap
    Pathfinder f(g);

    const PathResult r = f.find_path(at(2.5, 2.5), at(17.5, 2.5));
    REQUIRE(r.status == PathStatus::Found);
    require_legal_steps(g, at(2.5, 2.5), r);
    bool through_gap = false;
    for (const Vec3& wp : r.waypoints) {
        if (g.to_cell(wp)->x == 10) {
            CHECK(g.to_cell(wp)->y == 15);
            through_gap = true;
        }
    }
    CHECK(through_gap);
    CHECK(r.cost == reference_cost(g, Cell{2, 2}, Cell{17, 2}));
}

TEST_CASE("pathfinder: no cutting corners between diagonal blockers", "[nav][path]") {
    NavGrid g = open_grid(6, 6);
    g.set_blocked(Cell{2, 1}, true);
    g.set_blocked(Cell{1, 2}, true);  // (1,1) -> (2,2) would squeeze between two blocked corners
    Pathfinder f(g);
    const PathResult r = f.find_path(at(1.5, 1.5), at(2.5, 2.5));
    REQUIRE(r.status == PathStatus::Found);
    CHECK(r.waypoints.size() > 1);  // had to go around instead of the 1-step diagonal
    require_legal_steps(g, at(1.5, 1.5), r);
}

TEST_CASE("pathfinder: unreachable goal gives a partial path to the closest cell", "[nav][path]") {
    NavGrid g = open_grid(20, 20);
    (void)g.block_rect(10.0, 0.0, 11.0, 20.0);  // solid wall, no gap
    Pathfinder f(g);

    const PathResult r = f.find_path(at(2.5, 10.5), at(17.5, 10.5));
    REQUIRE(r.status == PathStatus::Partial);
    REQUIRE_FALSE(r.waypoints.empty());
    const Cell last = *g.to_cell(r.waypoints.back());
    CHECK(last.x == 9);  // stopped at the wall, on the goal's row
    CHECK(last.y == 10);
    require_legal_steps(g, at(2.5, 10.5), r);
}

TEST_CASE("pathfinder: blocked goal and blocked start are snapped to free cells", "[nav][path]") {
    NavGrid g = open_grid(20, 20);
    (void)g.block_rect(8.0, 8.0, 12.0, 12.0);  // a solid 4x4 block
    Pathfinder f(g);

    const PathResult to_block = f.find_path(at(2.5, 2.5), at(10.0, 10.0));  // goal inside the block
    REQUIRE(to_block.status == PathStatus::Found);
    CHECK(g.free(*g.to_cell(to_block.waypoints.back())));
    require_legal_steps(g, at(2.5, 2.5), to_block);

    const PathResult from_block = f.find_path(at(10.0, 10.0), at(2.5, 2.5));  // start inside the block
    REQUIRE(from_block.status == PathStatus::Found);
    CHECK(g.free(*g.to_cell(from_block.waypoints.front())));  // first move: out of the obstacle
    CHECK(from_block.waypoints.back() == at(2.5, 2.5));

    // Entirely walled in with no free cell within the snap radius: nothing to do.
    NavGrid solid = open_grid(10, 10);
    (void)solid.block_rect(0.0, 0.0, 10.0, 10.0);
    Pathfinder fs(solid);
    CHECK(fs.find_path(at(2.5, 2.5), at(7.5, 7.5)).status == PathStatus::NoPath);

    // Outside the map entirely.
    const NavGrid small = open_grid(5, 5);
    Pathfinder fo(small);
    CHECK(fo.find_path(at(100.0, 100.0), at(2.5, 2.5), PathOptions{.max_expansions = 1000, .snap_radius_cells = 3}).status ==
          PathStatus::NoPath);
}

TEST_CASE("pathfinder: an enclosed start cannot move", "[nav][path]") {
    NavGrid g = open_grid(9, 9);
    (void)g.block_rect(3.0, 3.0, 6.0, 6.0);
    g.set_blocked(Cell{4, 4}, false);  // a single free cell inside a solid block
    Pathfinder f(g);
    CHECK(f.find_path(at(4.5, 4.5), at(0.5, 0.5)).status == PathStatus::NoPath);
}

TEST_CASE("pathfinder: the expansion budget returns a partial path", "[nav][path]") {
    const NavGrid g = open_grid(100, 100);
    Pathfinder f(g);
    const PathResult full = f.find_path(at(1.5, 1.5), at(98.5, 98.5));
    REQUIRE(full.status == PathStatus::Found);

    PathOptions tight;
    tight.max_expansions = 5;
    const PathResult cut = f.find_path(at(1.5, 1.5), at(98.5, 98.5), tight);
    CHECK(cut.status == PathStatus::Partial);
    CHECK(cut.expansions <= 5);
    CHECK_FALSE(cut.waypoints.empty());
}

TEST_CASE("pathfinder: optimal against an independent Dijkstra on 200 random mazes", "[nav][path][stress]") {
    for (std::uint64_t seed = 1; seed <= 200; ++seed) {
        core::Rng rng(seed);
        NavGrid g = open_grid(24, 24);
        const std::uint32_t density = 15 + static_cast<std::uint32_t>(seed % 30);  // 15..44 %
        for (int y = 0; y < 24; ++y) {
            for (int x = 0; x < 24; ++x) {
                if (rng.next_u32() % 100 < density) g.set_blocked(Cell{x, y}, true);
            }
        }
        g.set_blocked(Cell{1, 1}, false);
        g.set_blocked(Cell{22, 22}, false);
        Pathfinder f(g);
        const PathResult r = f.find_path(at(1.5, 1.5), at(22.5, 22.5));
        const int want = reference_cost(g, Cell{1, 1}, Cell{22, 22});
        INFO("seed " << seed);
        if (want < 0) {
            CHECK(r.status != PathStatus::Found);
        } else {
            REQUIRE(r.status == PathStatus::Found);
            CHECK(r.cost == want);
            require_legal_steps(g, at(1.5, 1.5), r);
        }
    }
}

TEST_CASE("pathfinder: deterministic, and scratch reuse never changes an answer", "[nav][path]") {
    core::Rng rng(77);
    NavGrid g = open_grid(40, 40);
    for (int y = 0; y < 40; ++y) {
        for (int x = 0; x < 40; ++x) {
            if (rng.next_u32() % 100 < 28) g.set_blocked(Cell{x, y}, true);
        }
    }
    Pathfinder reused(g);
    for (int i = 0; i < 300; ++i) {
        const Vec3 a = at(static_cast<double>(rng.next_u32() % 40) + 0.5, static_cast<double>(rng.next_u32() % 40) + 0.5);
        const Vec3 b = at(static_cast<double>(rng.next_u32() % 40) + 0.5, static_cast<double>(rng.next_u32() % 40) + 0.5);
        Pathfinder fresh(g);
        const PathResult x = reused.find_path(a, b);
        const PathResult y = fresh.find_path(a, b);
        REQUIRE(x.status == y.status);
        REQUIRE(x.cost == y.cost);
        REQUIRE(x.expansions == y.expansions);
        REQUIRE(x.waypoints == y.waypoints);
    }
}

TEST_CASE("pathfinder: follows map edits between queries", "[nav][path]") {
    NavGrid g = open_grid(20, 5);
    Pathfinder f(g);
    CHECK(f.find_path(at(1.5, 2.5), at(18.5, 2.5)).cost == 170);
    (void)g.block_rect(10.0, 0.0, 11.0, 5.0);  // a wall appears
    CHECK(f.find_path(at(1.5, 2.5), at(18.5, 2.5)).status == PathStatus::Partial);
    (void)g.block_rect(10.0, 0.0, 11.0, 5.0, false);  // and goes away
    CHECK(f.find_path(at(1.5, 2.5), at(18.5, 2.5)).cost == 170);
}
