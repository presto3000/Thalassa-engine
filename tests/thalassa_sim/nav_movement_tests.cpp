#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "thalassa/core/serialization/byte_stream.hpp"
#include "thalassa/sim/commands.hpp"
#include "thalassa/sim/components.hpp"
#include "thalassa/sim/nav/nav_map.hpp"
#include "thalassa/sim/replay.hpp"
#include "thalassa/sim/replay_recording.hpp"
#include "thalassa/sim/sim_world.hpp"

using namespace thalassa;
using core::SimTick;
using core::ecs::Entity;
using core::math::Vec3;
using sim::Command;
using sim::SimWorld;
using sim::components::Destination;
using sim::components::Path;
using sim::components::Position;
using sim::components::Radius;
using sim::nav::Cell;
using sim::nav::NavGrid;
using sim::nav::NavMap;

namespace {

constexpr double kDt = 1.0 / 60.0;

Vec3 at(double x, double y) { return Vec3{x, y, 0.0}; }

// 40 x 40 m, 1 m cells, origin (0,0).
NavGrid open_grid(int w = 40, int h = 40) { return NavGrid(1.0, 0.0, 0.0, w, h); }

// A wall at x in [20, 21) from y = 0 up to (not including) `gap_from`; open above it.
NavGrid walled_grid(int gap_from = 30) {
    NavGrid g = open_grid();
    (void)g.block_rect(20.0, 0.0, 21.0, static_cast<double>(gap_from));
    return g;
}

struct Scene {
    explicit Scene(NavGrid grid, std::vector<double> radii = {}) : world(1) {
        map = NavMap::create(std::move(grid), std::move(radii));
        world.set_nav_map(map);
    }
    Entity spawn(Vec3 p, double radius = 0.0) {
        Command c = Command::spawn_unit(p, core::PlayerId{1}, 0, 100.0, 0.0, 0.0, 1.0);
        c.radius = radius;
        return sim::apply_commands(world.world(), {c}).front();
    }
    void tick(const std::vector<Command>& cmds = {}) {
        world.tick(SimTick{world.ticks_simulated()}, cmds);
    }
    Vec3 pos(Entity e) const { return world.world().get<Position>(e).value; }
    bool walking(Entity e) const { return world.world().has<Destination>(e); }
    bool base_free_at(Vec3 p) const {
        const auto c = map->base().to_cell(p);
        return c && map->base().free(*c);
    }

    SimWorld world;
    std::shared_ptr<const NavMap> map;
};

std::uint64_t checksum(const SimWorld& w) { return sim::world_checksum(w); }

}  // namespace

TEST_CASE("nav movement: a unit walks around a wall, arrives exactly, at constant speed", "[nav][move]") {
    Scene s(walled_grid());
    const Entity u = s.spawn(at(5.5, 5.5));
    const double speed = 6.0;
    s.tick({Command::move_to(u, at(35.5, 5.5), speed)});

    REQUIRE(s.world.world().has<Path>(u));
    const Path& path = s.world.world().get<Path>(u);
    REQUIRE(path.count >= 2);  // it has to bend around the wall

    // Length of the planned route, from the unit's start to the last waypoint.
    double route = 0.0;
    Vec3 prev = at(5.5, 5.5);
    for (int i = 0; i < path.count; ++i) {
        route += (path.points[i] - prev).length();
        prev = path.points[i];
    }
    CHECK(route > 30.0);  // longer than the straight 30 m: it detoured

    const double step = speed * kDt;
    Vec3 last = s.pos(u);
    int ticks = 1;
    while (s.walking(u) && ticks < 3000) {
        s.tick();
        ++ticks;
        const Vec3 now = s.pos(u);
        CHECK((now - last).length() <= step + 1e-9);  // never faster than its speed
        CHECK(s.base_free_at(now));                    // never inside the wall
        last = now;
    }
    CHECK_FALSE(s.walking(u));
    CHECK(s.pos(u) == at(35.5, 5.5));  // exactly on the goal
    CHECK_FALSE(s.world.world().has<Path>(u));
    // Time taken matches route length / speed (corners cost nothing).
    CHECK(ticks >= static_cast<int>(std::floor(route / step)));
    CHECK(ticks <= static_cast<int>(std::ceil(route / step)) + 2);
}

TEST_CASE("nav movement: without a nav map nothing changes (straight line)", "[nav][move]") {
    SimWorld w(1);
    CHECK(w.nav_hash() == 0);
    const Entity u = sim::apply_commands(w.world(), {Command::spawn_unit(at(5.5, 5.5), core::PlayerId{1}, 0, 100, 0, 0, 1)}).front();
    w.tick(SimTick{0}, {Command::move_to(u, at(35.5, 5.5), 6.0)});
    CHECK_FALSE(w.world().has<Path>(u));
    for (int i = 0; i < 100; ++i) w.tick(SimTick{w.ticks_simulated()});
    const Vec3 p = w.world().get<Position>(u).value;
    CHECK(p.y == 5.5);  // straight along x
    CHECK(p.x > 5.5);
}

TEST_CASE("nav movement: an unreachable goal ends at the closest point, then the unit rests", "[nav][move]") {
    Scene s(walled_grid(/*gap_from=*/40));  // wall from y=0 to the top: no way around
    const Entity u = s.spawn(at(5.5, 10.5));
    s.tick({Command::move_to(u, at(35.5, 10.5), 8.0)});
    REQUIRE(s.world.world().has<Path>(u));
    CHECK((s.world.world().get<Path>(u).flags & Path::kPartial) != 0);

    for (int i = 0; i < 1000 && s.walking(u); ++i) {
        s.tick();
        CHECK(s.base_free_at(s.pos(u)));
    }
    CHECK_FALSE(s.walking(u));
    const Vec3 p = s.pos(u);
    CHECK(p.x < 20.0);
    CHECK(p.x > 18.0);  // right up against the wall
    CHECK(p.y == 10.5);
}

TEST_CASE("nav movement: an order with no possible route is dropped", "[nav][move]") {
    NavGrid g = open_grid(10, 10);
    (void)g.block_rect(0.0, 0.0, 10.0, 10.0);  // everything blocked
    Scene s(std::move(g));
    const Entity u = s.spawn(at(5.5, 5.5));
    s.tick({Command::move_to(u, at(8.5, 8.5), 5.0)});
    CHECK_FALSE(s.walking(u));
    CHECK_FALSE(s.world.world().has<Path>(u));
    CHECK(s.pos(u) == at(5.5, 5.5));
}

TEST_CASE("nav movement: unit size decides which gaps a unit may use", "[nav][move][size]") {
    NavGrid g = open_grid();
    (void)g.block_rect(20.0, 0.0, 21.0, 40.0);  // solid wall...
    g.set_blocked(Cell{20, 20}, false);          // ...with a single 1 m gap at y = 20
    Scene s(std::move(g), {0.4, 0.7});

    const Entity thin = s.spawn(at(5.5, 20.5), 0.3);  // class radius 0.4: fits
    const Entity fat = s.spawn(at(5.5, 25.5), 0.6);   // class radius 0.7: does not
    s.tick({Command::move_to(thin, at(35.5, 20.5), 8.0), Command::move_to(fat, at(35.5, 25.5), 8.0)});
    for (int i = 0; i < 2000 && (s.walking(thin) || s.walking(fat)); ++i) s.tick();

    CHECK(s.pos(thin) == at(35.5, 20.5));
    CHECK(s.pos(fat).x < 20.0);  // stuck on its side of the wall, as close as it could get
    CHECK(s.world.world().get<Radius>(fat).value == 0.6);
}

TEST_CASE("nav movement: routes longer than the Path capacity are replanned and still arrive", "[nav][move]") {
    // A serpentine: 12 walls, each with its gap alternately at the bottom and the top.
    NavGrid g(1.0, 0.0, 0.0, 64, 30);
    for (int k = 0; k < 12; ++k) {
        const double x = 4.0 + 4.0 * k;
        const bool gap_low = (k % 2) == 0;
        (void)g.block_rect(x, gap_low ? 4.0 : 0.0, x + 1.0, gap_low ? 30.0 : 26.0);
    }
    Scene s(std::move(g));
    const Entity u = s.spawn(at(1.5, 1.5));
    s.tick({Command::move_to(u, at(60.5, 15.5), 10.0)});

    REQUIRE(s.world.world().has<Path>(u));
    const Path& first = s.world.world().get<Path>(u);
    CHECK((first.flags & Path::kTruncated) != 0);  // did not fit
    CHECK(first.count == Path::kMaxWaypoints);

    std::uint64_t searches_before = s.world.path_planner()->searches();
    int ticks = 0;
    while (s.walking(u) && ticks < 5000) {
        s.tick();
        ++ticks;
        REQUIRE(s.base_free_at(s.pos(u)));
    }
    CHECK(s.pos(u) == at(60.5, 15.5));
    CHECK(s.world.path_planner()->searches() > searches_before);  // it asked for the next stretch at least once
}

TEST_CASE("nav movement: a new order replaces the old route", "[nav][move]") {
    Scene s(walled_grid());
    const Entity u = s.spawn(at(5.5, 5.5));
    s.tick({Command::move_to(u, at(35.5, 5.5), 6.0)});
    for (int i = 0; i < 30; ++i) s.tick();
    s.tick({Command::move_to(u, at(5.5, 35.5), 6.0)});  // change of mind: somewhere else entirely
    for (int i = 0; i < 2000 && s.walking(u); ++i) s.tick();
    CHECK(s.pos(u) == at(5.5, 35.5));
}

TEST_CASE("nav movement: a snapshot taken mid-walk continues identically", "[nav][move][snapshot]") {
    auto build = [](Scene& s, std::vector<Entity>& units) {
        for (int i = 0; i < 5; ++i) units.push_back(s.spawn(at(3.5 + i, 4.5 + 2.0 * i), i % 2 == 0 ? 0.0 : 0.5));
    };
    const NavGrid grid = walled_grid();

    Scene a(grid, {0.5});
    std::vector<Entity> ua;
    build(a, ua);
    std::vector<Command> orders;
    for (std::size_t i = 0; i < ua.size(); ++i) orders.push_back(Command::move_to(ua[i], at(35.5, 3.5 + 6.0 * static_cast<double>(i)), 7.0));
    a.tick(orders);
    for (int i = 0; i < 90; ++i) a.tick();

    core::serialization::ByteWriter snap;
    sim::save_world(a.world, snap);

    Scene b(grid, {0.5});  // same map, fresh world
    core::serialization::ByteReader reader(snap.bytes());
    sim::load_world(b.world, reader);
    CHECK(checksum(a.world) == checksum(b.world));

    for (int i = 0; i < 600; ++i) {
        a.tick();
        b.tick();
    }
    CHECK(checksum(a.world) == checksum(b.world));
    for (const Entity e : ua) CHECK_FALSE(a.walking(e));  // everyone made it
}

TEST_CASE("nav movement: the same scenario twice gives identical worlds", "[nav][move][determinism]") {
    auto run = [] {
        Scene s(walled_grid(), {0.5});
        std::vector<Command> orders;
        std::vector<Entity> us;
        for (int i = 0; i < 12; ++i) us.push_back(s.spawn(at(2.5 + i * 1.5, 3.5 + (i % 4) * 5.0), (i % 3) * 0.25));
        for (std::size_t i = 0; i < us.size(); ++i) orders.push_back(Command::move_to(us[i], at(34.5, 2.5 + 3.0 * static_cast<double>(i)), 4.0 + static_cast<double>(i % 5)));
        s.tick(orders);
        for (int i = 0; i < 1500; ++i) s.tick();
        return checksum(s.world);
    };
    CHECK(run() == run());
}

TEST_CASE("nav map: size classes, hash and serialization", "[nav][map]") {
    NavGrid g = open_grid(20, 20);
    g.set_blocked(Cell{10, 10}, true);
    const auto m = NavMap::create(g, {0.7, 0.3, 0.7, -1.0, 0.0});  // unsorted, duplicate and bad radii are cleaned up
    REQUIRE(m->class_count() == 3);
    CHECK(m->class_radius(0) == 0.0);
    CHECK(m->class_radius(1) == 0.3);
    CHECK(m->class_radius(2) == 0.7);
    CHECK(m->class_for_radius(0.0) == 0);
    CHECK(m->class_for_radius(0.1) == 1);
    CHECK(m->class_for_radius(0.3) == 1);
    CHECK(m->class_for_radius(0.31) == 2);
    CHECK(m->class_for_radius(5.0) == 2);  // bigger than every class: the largest
    CHECK(m->grid(2).blocked(Cell{9, 10}));
    CHECK(m->grid(0).free(Cell{9, 10}));

    CHECK(NavMap::create(g, {0.3, 0.7})->hash() == m->hash());
    CHECK(NavMap::create(g, {0.3})->hash() != m->hash());
    NavGrid other = g;
    other.set_blocked(Cell{0, 0}, true);
    CHECK(NavMap::create(other, {0.3, 0.7})->hash() != m->hash());

    const auto bytes = m->serialize();
    const auto back = NavMap::deserialize(bytes);
    REQUIRE(back != nullptr);
    CHECK(back->hash() == m->hash());
    CHECK(back->class_count() == 3);
    for (std::size_t n = 0; n < bytes.size(); n += 7) {
        CHECK(NavMap::deserialize(std::span<const std::byte>(bytes.data(), n)) == nullptr);  // truncations
    }
    auto bad = bytes;
    bad[0] = std::byte{0};
    CHECK(NavMap::deserialize(bad) == nullptr);
}

TEST_CASE("replay file: unit type and radius survive recording, saving and playback", "[nav][replay]") {
    sim::ReplayRecorder rec(9, kDt, 30);
    Command spawn = Command::spawn_unit(at(1.0, 1.0), core::PlayerId{1}, 0, 50.0, 5.0, 3.0, 1.0).with_radius(0.75);
    spawn.unit_type = 7;
    const auto spawned = rec.submit(spawn);
    REQUIRE(spawned.size() == 1);
    for (int i = 0; i < 60; ++i) rec.step();
    const sim::ReplayRecording recording = rec.finish();

    core::serialization::ByteWriter w;
    recording.serialize(w);
    sim::ReplayRecording loaded;
    core::serialization::ByteReader r(w.bytes());
    REQUIRE(loaded.deserialize(r));

    sim::ReplayPlayer player(loaded);
    while (player.step_tick()) {}
    CHECK(player.final_checksum_matches());
    bool found = false;
    player.sim().world().each<Radius>([&](Entity, const Radius& rad) { found = rad.value == 0.75; });
    CHECK(found);
}
