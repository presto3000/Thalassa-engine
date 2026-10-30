#include <cmath>
#include <cstring>
#include <algorithm>
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

// Step-1 tests look at the raw grid path.
PathOptions raw() {
    PathOptions o;
    o.smooth = false;
    return o;
}

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

    const PathResult straight = f.find_path(at(1.5, 1.5), at(8.5, 1.5), raw());
    REQUIRE(straight.status == PathStatus::Found);
    CHECK(straight.cost == 70);
    REQUIRE(straight.waypoints.size() == 7);
    CHECK(straight.waypoints.back() == at(8.5, 1.5));  // exact goal point
    require_legal_steps(g, at(1.5, 1.5), straight);

    const PathResult diag = f.find_path(at(1.5, 1.5), at(6.2, 6.9), raw());
    REQUIRE(diag.status == PathStatus::Found);
    CHECK(diag.cost == 5 * 14);
    CHECK(diag.waypoints.back() == at(6.2, 6.9));

    // Start and goal in the same cell: one waypoint, the goal itself.
    const PathResult same = f.find_path(at(3.1, 3.1), at(3.9, 3.4), raw());
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

    const PathResult r = f.find_path(at(2.5, 2.5), at(17.5, 2.5), raw());
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
    const PathResult r = f.find_path(at(1.5, 1.5), at(2.5, 2.5), raw());
    REQUIRE(r.status == PathStatus::Found);
    CHECK(r.waypoints.size() > 1);  // had to go around instead of the 1-step diagonal
    require_legal_steps(g, at(1.5, 1.5), r);
}

TEST_CASE("pathfinder: unreachable goal gives a partial path to the closest cell", "[nav][path]") {
    NavGrid g = open_grid(20, 20);
    (void)g.block_rect(10.0, 0.0, 11.0, 20.0);  // solid wall, no gap
    Pathfinder f(g);

    const PathResult r = f.find_path(at(2.5, 10.5), at(17.5, 10.5), raw());
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

    const PathResult to_block = f.find_path(at(2.5, 2.5), at(10.0, 10.0), raw());  // goal inside the block
    REQUIRE(to_block.status == PathStatus::Found);
    CHECK(g.free(*g.to_cell(to_block.waypoints.back())));
    require_legal_steps(g, at(2.5, 2.5), to_block);

    const PathResult from_block = f.find_path(at(10.0, 10.0), at(2.5, 2.5), raw());  // start inside the block
    REQUIRE(from_block.status == PathStatus::Found);
    CHECK(g.free(*g.to_cell(from_block.waypoints.front())));  // first move: out of the obstacle
    CHECK(from_block.waypoints.back() == at(2.5, 2.5));

    // Entirely walled in with no free cell within the snap radius: nothing to do.
    NavGrid solid = open_grid(10, 10);
    (void)solid.block_rect(0.0, 0.0, 10.0, 10.0);
    Pathfinder fs(solid);
    CHECK(fs.find_path(at(2.5, 2.5), at(7.5, 7.5), raw()).status == PathStatus::NoPath);

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
    CHECK(f.find_path(at(4.5, 4.5), at(0.5, 0.5), raw()).status == PathStatus::NoPath);
}

TEST_CASE("pathfinder: the expansion budget returns a partial path", "[nav][path]") {
    const NavGrid g = open_grid(100, 100);
    Pathfinder f(g);
    const PathResult full = f.find_path(at(1.5, 1.5), at(98.5, 98.5), raw());
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
        const PathResult r = f.find_path(at(1.5, 1.5), at(22.5, 22.5), raw());
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
        const PathResult x = reused.find_path(a, b, raw());
        const PathResult y = fresh.find_path(a, b, raw());
        REQUIRE(x.status == y.status);
        REQUIRE(x.cost == y.cost);
        REQUIRE(x.expansions == y.expansions);
        REQUIRE(x.waypoints == y.waypoints);
    }
}

TEST_CASE("pathfinder: follows map edits between queries", "[nav][path]") {
    NavGrid g = open_grid(20, 5);
    Pathfinder f(g);
    CHECK(f.find_path(at(1.5, 2.5), at(18.5, 2.5), raw()).cost == 170);
    (void)g.block_rect(10.0, 0.0, 11.0, 5.0);  // a wall appears
    CHECK(f.find_path(at(1.5, 2.5), at(18.5, 2.5), raw()).status == PathStatus::Partial);
    (void)g.block_rect(10.0, 0.0, 11.0, 5.0, false);  // and goes away
    CHECK(f.find_path(at(1.5, 2.5), at(18.5, 2.5), raw()).cost == 170);
}

// ---------------------------------------------------------------------------
// Step 2: line of sight, smoothing, unit size
// ---------------------------------------------------------------------------
namespace {

double path_length(const Vec3& start, const std::vector<Vec3>& wps) {
    double len = 0.0;
    Vec3 prev = start;
    for (const Vec3& w : wps) {
        len += (w - prev).length();
        prev = w;
    }
    return len;
}

// Reference for segment_clear: sample densely; any sample on a blocked cell => not clear.
bool sampled_clear(const NavGrid& g, const Vec3& a, const Vec3& b) {
    const int n = 4000;
    for (int i = 0; i <= n; ++i) {
        const double t = static_cast<double>(i) / n;
        const Vec3 p{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, 0.0};
        const auto c = g.to_cell(p);
        if (!c || g.blocked(*c)) return false;
    }
    return true;
}

NavGrid random_grid(std::uint64_t seed, int w, int h, std::uint32_t density_percent) {
    core::Rng rng(seed);
    NavGrid g = open_grid(w, h);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (rng.next_u32() % 100 < density_percent) g.set_blocked(Cell{x, y}, true);
        }
    }
    return g;
}

}  // namespace

TEST_CASE("segment_clear: basics, corners and edges", "[nav][los]") {
    NavGrid g = open_grid(10, 10);
    CHECK(g.segment_clear(at(0.5, 0.5), at(9.5, 9.5)));
    CHECK(g.segment_clear(at(0.5, 0.5), at(0.5, 0.5)));
    CHECK_FALSE(g.segment_clear(at(0.5, 0.5), at(10.5, 0.5)));  // ends outside the map
    CHECK_FALSE(g.segment_clear(at(-1.0, 0.5), at(2.5, 0.5)));  // starts outside

    g.set_blocked(Cell{5, 5}, true);
    CHECK_FALSE(g.segment_clear(at(0.5, 5.5), at(9.5, 5.5)));  // straight through the block
    CHECK(g.segment_clear(at(0.5, 4.5), at(9.5, 4.5)));        // the row below is free
    CHECK_FALSE(g.segment_clear(at(5.5, 5.5), at(8.5, 8.5)));  // starts inside it
    // Clips only the block's corner sliver: still blocked (conservative).
    CHECK_FALSE(g.segment_clear(at(4.0, 6.2), at(6.2, 4.0)));
    // Passes just below its corner (the x + y = 10 line would touch it).
    CHECK(g.segment_clear(at(3.0, 6.5), at(6.5, 3.0)));  // x + y = 9.5: misses the block (x + y >= 10)

    // Exactly through a grid corner between two diagonal blockers: not clear.
    NavGrid d = open_grid(6, 6);
    d.set_blocked(Cell{2, 1}, true);
    d.set_blocked(Cell{1, 2}, true);
    CHECK_FALSE(d.segment_clear(at(1.5, 1.5), at(2.5, 2.5)));
    // Exactly through a corner with everything free: fine.
    NavGrid f = open_grid(6, 6);
    CHECK(f.segment_clear(at(1.5, 1.5), at(3.5, 3.5)));
    // Corner crossing but one side blocked: not clear.
    f.set_blocked(Cell{2, 1}, true);
    CHECK_FALSE(f.segment_clear(at(1.5, 1.5), at(3.5, 3.5)));
}

TEST_CASE("segment_clear: never says clear when a sampled walk hits an obstacle; symmetric", "[nav][los][stress]") {
    for (std::uint64_t seed = 1; seed <= 60; ++seed) {
        const NavGrid g = random_grid(seed, 20, 20, 20 + static_cast<std::uint32_t>(seed % 20));
        core::Rng rng(seed * 7919);
        for (int i = 0; i < 150; ++i) {
            const Vec3 a = at(rng.next_u32() % 2000 / 100.0, rng.next_u32() % 2000 / 100.0);
            const Vec3 b = at(rng.next_u32() % 2000 / 100.0, rng.next_u32() % 2000 / 100.0);
            const bool ab = g.segment_clear(a, b);
            INFO("seed " << seed << " i " << i);
            CHECK(ab == g.segment_clear(b, a));
            if (ab) CHECK(sampled_clear(g, a, b));  // soundness: no false "clear"
        }
    }
}

TEST_CASE("smoothing: open field collapses to the destination", "[nav][smooth]") {
    const NavGrid g = open_grid(30, 30);
    Pathfinder f(g);
    const PathResult r = f.find_path(at(1.5, 1.5), at(25.2, 17.7));
    REQUIRE(r.status == PathStatus::Found);
    REQUIRE(r.waypoints.size() == 1);
    CHECK(r.waypoints[0] == at(25.2, 17.7));

    PathOptions o;
    o.keep_raw = true;
    const PathResult both = f.find_path(at(1.5, 1.5), at(25.2, 17.7), o);
    CHECK(both.raw_waypoints.size() > both.waypoints.size());
    CHECK(both.raw_waypoints.back() == both.waypoints.back());
}

TEST_CASE("smoothing: bends only around obstacles", "[nav][smooth]") {
    NavGrid g = open_grid(30, 30);
    (void)g.block_rect(14.0, 5.0, 16.0, 30.0);  // wall with open ground below y = 5
    Pathfinder f(g);
    PathOptions o;
    o.keep_raw = true;
    const PathResult r = f.find_path(at(3.5, 20.5), at(26.5, 20.5), o);
    REQUIRE(r.status == PathStatus::Found);
    CHECK(r.waypoints.size() <= 4);
    CHECK(r.waypoints.size() < r.raw_waypoints.size());
    CHECK(r.waypoints.back() == at(26.5, 20.5));
    Vec3 prev = at(3.5, 20.5);
    for (const Vec3& wp : r.waypoints) {
        CHECK(g.segment_clear(prev, wp));
        prev = wp;
    }
    CHECK(path_length(at(3.5, 20.5), r.waypoints) <= path_length(at(3.5, 20.5), r.raw_waypoints) + 1e-9);
}

TEST_CASE("smoothing: on 100 random mazes it is legal, never longer, never more points", "[nav][smooth][stress]") {
    for (std::uint64_t seed = 1; seed <= 100; ++seed) {
        NavGrid g = random_grid(seed, 30, 30, 15 + static_cast<std::uint32_t>(seed % 25));
        g.set_blocked(Cell{1, 1}, false);
        g.set_blocked(Cell{28, 28}, false);
        Pathfinder f(g);
        PathOptions o;
        o.keep_raw = true;
        const Vec3 start = at(1.5, 1.5);
        const PathResult r = f.find_path(start, at(28.5, 28.5), o);
        INFO("seed " << seed);
        if (r.waypoints.empty()) continue;
        CHECK(r.waypoints.size() <= r.raw_waypoints.size());
        CHECK(path_length(start, r.waypoints) <= path_length(start, r.raw_waypoints) + 1e-9);
        CHECK(r.waypoints.back() == r.raw_waypoints.back());
        Vec3 prev = start;
        for (const Vec3& wp : r.waypoints) {
            CHECK(g.segment_clear(prev, wp));
            CHECK(sampled_clear(g, prev, wp));
            prev = wp;
        }
    }
}

TEST_CASE("smoothing: a unit starting inside an obstacle keeps a legal first move", "[nav][smooth]") {
    NavGrid g = open_grid(20, 20);
    (void)g.block_rect(8.0, 8.0, 12.0, 12.0);
    Pathfinder f(g);
    const PathResult r = f.find_path(at(10.0, 10.0), at(2.5, 2.5));
    REQUIRE(r.status == PathStatus::Found);
    REQUIRE_FALSE(r.waypoints.empty());
    CHECK(g.free(*g.to_cell(r.waypoints.front())));
    CHECK(r.waypoints.back() == at(2.5, 2.5));
}

TEST_CASE("explored cells can be recorded for debugging, and cost nothing when off", "[nav][debug]") {
    NavGrid g = open_grid(30, 30);
    (void)g.block_rect(14.0, 5.0, 16.0, 30.0);
    Pathfinder f(g);
    PathOptions o;
    o.record_explored = true;
    const PathResult with = f.find_path(at(3.5, 20.5), at(26.5, 20.5), o);
    const PathResult without = f.find_path(at(3.5, 20.5), at(26.5, 20.5));
    CHECK(with.explored.size() == with.expansions);
    CHECK(without.explored.empty());
    CHECK(with.waypoints == without.waypoints);  // recording never changes the answer
    for (const Cell& c : with.explored) CHECK(g.free(c));
    // Distinct cells only: A* expands each cell at most once.
    std::set<std::pair<int, int>> seen;
    for (const Cell& c : with.explored) seen.insert({c.x, c.y});
    CHECK(seen.size() == with.explored.size());
}

TEST_CASE("inflation: grows obstacles and the border by the unit radius", "[nav][inflate]") {
    NavGrid g = open_grid(15, 15);
    g.set_blocked(Cell{7, 7}, true);

    CHECK(g.inflated(0.0) == g);   // nothing to do
    CHECK(g.inflated(-1.0) == g);

    // Radius 1 cell: the 8 neighbours of the block (distance .5 cells) are blocked, 2 cells away is not.
    const NavGrid r1 = g.inflated(1.0);
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) CHECK(r1.blocked(Cell{7 + dx, 7 + dy}));
    }
    CHECK(r1.free(Cell{9, 7}));
    CHECK(r1.free(Cell{7, 5}));
    CHECK(r1.free(Cell{0, 0}) == false);  // the border cell is within 1 of the outside
    CHECK(r1.free(Cell{1, 1}));

    // Radius 0.5 cell exactly touches: still free next to the block. Border cells too.
    const NavGrid r05 = g.inflated(0.5);
    CHECK(r05.free(Cell{8, 7}));
    CHECK(r05.free(Cell{0, 0}));
    // Slightly more than half a cell blocks the adjacent cells and the border ring.
    const NavGrid r06 = g.inflated(0.6);
    CHECK(r06.blocked(Cell{8, 7}));
    CHECK(r06.blocked(Cell{0, 5}));
    CHECK(r06.free(Cell{1, 5}));

    // Free-space distance check against brute force (center to blocked square), several radii.
    for (const double radius : {0.3, 0.7, 1.0, 1.6, 2.4}) {
        const NavGrid inf = g.inflated(radius);
        for (int y = 0; y < 15; ++y) {
            for (int x = 0; x < 15; ++x) {
                if (g.blocked(Cell{x, y})) continue;
                bool want_blocked = false;
                for (int by = -5; by <= 20 && !want_blocked; ++by) {
                    for (int bx = -5; bx <= 20 && !want_blocked; ++bx) {
                        if (!g.blocked(Cell{bx, by})) continue;
                        const double ex = std::max(0.0, std::abs(bx - x) - 0.5);
                        const double ey = std::max(0.0, std::abs(by - y) - 0.5);
                        if (ex * ex + ey * ey < radius * radius) want_blocked = true;
                    }
                }
                INFO("r " << radius << " cell " << x << "," << y);
                CHECK(inf.blocked(Cell{x, y}) == want_blocked);
            }
        }
    }
}

TEST_CASE("unit size: a gap one cell wide admits a thin unit but not a fat one", "[nav][inflate][smooth]") {
    NavGrid g = open_grid(20, 12);
    (void)g.block_rect(10.0, 0.0, 11.0, 12.0);  // wall at column 10
    g.set_blocked(Cell{10, 6}, false);           // one-cell gap (1 m wide)

    const NavGrid thin = g.inflated(0.4);   // diameter 0.8 m < 1 m gap
    const NavGrid fat = g.inflated(0.7);    // diameter 1.4 m > 1 m gap
    Pathfinder fthin(thin), ffat(fat);

    const PathResult a = fthin.find_path(at(2.5, 6.5), at(17.5, 6.5));
    CHECK(a.status == PathStatus::Found);
    const PathResult b = ffat.find_path(at(2.5, 6.5), at(17.5, 6.5));
    CHECK(b.status == PathStatus::Partial);  // can only get as close as the wall allows
    CHECK(b.waypoints.back().x < 10.0);

    // A fat unit given a 3-cell gap does get through.
    g.set_blocked(Cell{10, 5}, false);
    g.set_blocked(Cell{10, 7}, false);
    const NavGrid fat2 = g.inflated(0.7);
    Pathfinder ffat2(fat2);
    const PathResult c = ffat2.find_path(at(2.5, 6.5), at(17.5, 6.5));
    CHECK(c.status == PathStatus::Found);
}

TEST_CASE("unit size: smoothed paths keep their distance from walls", "[nav][inflate][smooth]") {
    NavGrid g = open_grid(40, 40);
    (void)g.block_rect(18.0, 0.0, 20.0, 30.0);  // wall, open above y = 30
    const double radius = 0.9;
    const NavGrid inf = g.inflated(radius);
    Pathfinder f(inf);
    const Vec3 start = at(5.5, 5.5);
    const PathResult r = f.find_path(start, at(33.5, 5.5));
    REQUIRE(r.status == PathStatus::Found);

    // Sample the whole walk: distance to every blocked square of the REAL map must stay near
    // the radius (center-based freedom allows up to 0.71 x cell of slack, see inflated()).
    Vec3 prev = start;
    for (const Vec3& wp : r.waypoints) {
        for (int i = 0; i <= 200; ++i) {
            const double t = i / 200.0;
            const Vec3 p{prev.x + (wp.x - prev.x) * t, prev.y + (wp.y - prev.y) * t, 0.0};
            for (int y = 0; y < 40; ++y) {
                for (int x = 0; x < 40; ++x) {
                    if (g.free(Cell{x, y})) continue;
                    const double ex = std::max({0.0, x - p.x, p.x - (x + 1.0)});
                    const double ey = std::max({0.0, y - p.y, p.y - (y + 1.0)});
                    REQUIRE(std::sqrt(ex * ex + ey * ey) >= radius - 0.71 - 1e-9);
                }
            }
        }
        prev = wp;
    }
}

TEST_CASE("smoothed pathfinding is deterministic", "[nav][smooth]") {
    const NavGrid g = random_grid(5, 50, 50, 25);
    Pathfinder reused(g);
    core::Rng rng(11);
    for (int i = 0; i < 200; ++i) {
        const Vec3 a = at(rng.next_u32() % 5000 / 100.0, rng.next_u32() % 5000 / 100.0);
        const Vec3 b = at(rng.next_u32() % 5000 / 100.0, rng.next_u32() % 5000 / 100.0);
        Pathfinder fresh(g);
        const PathResult x = reused.find_path(a, b);
        const PathResult y = fresh.find_path(a, b);
        REQUIRE(x.status == y.status);
        REQUIRE(x.waypoints == y.waypoints);
    }
}
