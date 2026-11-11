#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "thalassa/sim/commands.hpp"
#include "thalassa/sim/components.hpp"
#include "thalassa/sim/nav/nav_debug.hpp"
#include "thalassa/sim/replay_recording.hpp"
#include "thalassa/sim/sim_world.hpp"

using namespace thalassa;
using namespace thalassa::sim::nav;
using namespace thalassa::sim::nav::debug;
using core::SimTick;
using core::math::Vec3;
using sim::Command;
using sim::SimWorld;

namespace {

// 40 x 40, wall at x in [20,21) from y 0..30.
std::shared_ptr<const NavMap> walled_map() {
    NavGrid g(1.0, 0.0, 0.0, 40, 40);
    (void)g.block_rect(20.0, 0.0, 21.0, 30.0);
    return NavMap::create(std::move(g), {0.4, 1.2});
}

core::ecs::Entity spawn(SimWorld& w, Vec3 p, double radius = 0.0) {
    Command c = Command::spawn_unit(p, core::PlayerId{1}, 0, 100.0, 0.0, 0.0, 1.0);
    c.radius = radius;
    return sim::apply_commands(w.world(), {c}).front();
}

}  // namespace

TEST_CASE("nav debug: master switch gates every layer and is off by default", "[nav][debug]") {
    NavDebugSettings s;
    CHECK_FALSE(s.master());
    CHECK_FALSE(s.any());
    for (std::size_t i = 0; i < kLayerCount; ++i) CHECK_FALSE(s.enabled(static_cast<Layer>(i)));

    s.set_master(true);
    CHECK(s.enabled(Layer::Paths));  // default layers
    CHECK_FALSE(s.enabled(Layer::Explored));
    s.set_layer(Layer::Explored, true);
    CHECK(s.enabled(Layer::Explored));

    s.kill();
    for (std::size_t i = 0; i < kLayerCount; ++i) CHECK_FALSE(s.enabled(static_cast<Layer>(i)));
    CHECK(s.layer_on(Layer::Explored));  // layer state survives the master switch
    s.set_master(true);
    CHECK(s.enabled(Layer::Explored));

    s.set_all_layers(false);
    CHECK_FALSE(s.any());
    s.set_all_layers(true);
    for (std::size_t i = 0; i < kLayerCount; ++i) CHECK(s.enabled(static_cast<Layer>(i)));
}

TEST_CASE("nav debug: layers are switched by console name and the checklist shows it", "[nav][debug]") {
    NavDebugSettings s;
    CHECK(s.set_by_name("master", true));
    CHECK(s.set_by_name("grid", true));
    CHECK(s.enabled(Layer::Grid));
    CHECK(s.set_by_name("grid", false));
    CHECK_FALSE(s.enabled(Layer::Grid));
    CHECK(s.toggle_by_name("coords"));
    CHECK(s.enabled(Layer::CellCoords));
    CHECK_FALSE(s.set_by_name("nonsense", true));
    CHECK_FALSE(s.toggle_by_name(""));
    CHECK(s.set_by_name("all", true));
    CHECK(s.enabled(Layer::Panel));

    for (std::size_t i = 0; i < kLayerCount; ++i) {  // names round-trip
        Layer back{};
        REQUIRE(parse_layer(layer_name(static_cast<Layer>(i)), back));
        CHECK(static_cast<std::size_t>(back) == i);
    }

    NavDebugSettings fresh;
    const auto lines = fresh.checklist();
    REQUIRE(lines.size() == kLayerCount + 1);
    CHECK(lines[0] == "[ ] master (nav debug)");
    CHECK(lines[1] == "[ ] 1 grid");
    CHECK(lines[3] == "[x] 3 paths");
}

TEST_CASE("nav debug: blocked and inflated cells come out as merged runs", "[nav][debug]") {
    const auto map = walled_map();
    const auto runs = blocked_runs(map->base());
    std::size_t cells = 0;
    for (const CellRun& r : runs) {
        CHECK(r.x_begin == 20);
        CHECK(r.x_end == 21);
        cells += static_cast<std::size_t>(r.x_end - r.x_begin);
    }
    CHECK(runs.size() == 30);  // one 1-cell run per row of the wall
    CHECK(cells == 30);

    CHECK(inflated_runs(*map, 0).empty());
    const auto fat = inflated_runs(*map, 2);  // radius 1.2 (class 1, r=0.4, adds nothing on 1 m cells)
    CHECK(inflated_runs(*map, 1).empty());
    CHECK_FALSE(fat.empty());
    for (const CellRun& r : fat) {  // never overlaps what the base map already blocks
        for (std::int32_t x = r.x_begin; x < r.x_end; ++x) CHECK(map->base().free(Cell{x, r.y}));
    }
    CHECK(inflated_runs(*map, 99).empty());

    NavGrid open(1.0, 0.0, 0.0, 6, 3);
    (void)open.block_rect(1.0, 1.0, 5.0, 2.0);
    const auto r2 = blocked_runs(open);
    REQUIRE(r2.size() == 1);
    CHECK(r2[0].y == 1);
    CHECK(r2[0].x_begin == 1);
    CHECK(r2[0].x_end == 5);
}

TEST_CASE("nav debug: routes of moving units are collected without touching the sim", "[nav][debug]") {
    SimWorld w(1);
    w.set_nav_map(walled_map());
    const auto a = spawn(w, {10.0, 10.0, 0.0});
    const auto b = spawn(w, {5.0, 5.0, 0.0}, 0.3);
    (void)b;  // idle
    CHECK(collect_routes(w).empty());

    w.tick(SimTick{0}, {Command::move_to(a, {30.0, 10.0, 0.0}, 6.0)});
    const std::uint64_t before = sim::world_checksum(w);
    const auto routes = collect_routes(w);
    REQUIRE(routes.size() == 1);
    CHECK(routes[0].entity == a);
    CHECK(routes[0].has_destination);
    CHECK(routes[0].destination.x == 30.0);
    CHECK_FALSE(routes[0].waypoints.empty());
    CHECK_FALSE(routes[0].partial);
    CHECK(routes[0].waypoints.back().x == 30.0);
    CHECK(std::any_of(routes[0].waypoints.begin(), routes[0].waypoints.end(), [](const Vec3& p) { return p.y >= 29.0; }));
    CHECK(sim::world_checksum(w) == before);  // read-only

    // Unreachable goal: the route is marked partial.
    NavGrid g(1.0, 0.0, 0.0, 20, 20);
    (void)g.block_rect(10.0, 0.0, 11.0, 20.0);  // full-height wall
    SimWorld w2(1);
    w2.set_nav_map(NavMap::create(std::move(g)));
    const auto c = spawn(w2, {2.0, 10.0, 0.0});
    w2.tick(SimTick{0}, {Command::move_to(c, {18.0, 10.0, 0.0}, 5.0)});
    const auto r2 = collect_routes(w2);
    REQUIRE(r2.size() == 1);
    CHECK(r2[0].partial);
    CHECK(summarize(w2).partial_paths == 1);
}

TEST_CASE("nav debug: probe_path reports the explored cells, raw and smoothed route", "[nav][debug]") {
    const auto map = walled_map();
    const PathResult r = probe_path(*map, {10.0, 10.0, 0.0}, {30.0, 10.0, 0.0});
    REQUIRE(r.status == PathStatus::Found);
    CHECK_FALSE(r.explored.empty());
    CHECK(r.expansions == r.explored.size());
    CHECK(r.raw_waypoints.size() > r.waypoints.size());  // smoothing removed grid zig-zag
    CHECK(r.waypoints.back().x == 30.0);

    // A fat unit searches the inflated grid: the route keeps more distance from the wall.
    const PathResult fat = probe_path(*map, {10.0, 10.0, 0.0}, {30.0, 10.0, 0.0}, 1.1);
    REQUIRE(fat.status == PathStatus::Found);
    CHECK(fat.cost >= r.cost);

    SimWorld w(1);
    w.set_nav_map(map);
    const auto summary = summarize(w, &r);
    CHECK(summary.has_map);
    CHECK(summary.map_hash == map->hash());
    CHECK(summary.blocked_cells == 30);
    CHECK(summary.size_classes == 3);
    CHECK(summary.probe_expansions == r.expansions);
    const auto lines = summary_lines(summary);
    CHECK(lines.size() == 5);
    CHECK(lines[0].starts_with("nav map "));

    SimWorld none(1);
    const auto no_map = summary_lines(summarize(none));
    REQUIRE(no_map.size() == 1);
    CHECK(no_map[0].starts_with("nav: no map"));
}
