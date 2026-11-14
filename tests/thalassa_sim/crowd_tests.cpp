#include <algorithm>
#include <cmath>
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
using sim::components::MoveProgress;
using sim::components::Position;
using sim::nav::NavGrid;
using sim::nav::NavMap;

namespace {

Vec3 at(double x, double y) { return Vec3{x, y, 0.0}; }

std::shared_ptr<const NavMap> open_map(int size = 40) {
    return NavMap::create(NavGrid(1.0, 0.0, 0.0, size, size), {0.5});
}

struct Scene {
    explicit Scene(std::shared_ptr<const NavMap> map = open_map()) : world(1) {
        if (map) world.set_nav_map(std::move(map));
    }
    Entity spawn(Vec3 p, double radius = 0.5) {
        Command c = Command::spawn_unit(p, core::PlayerId{1}, 0, 100.0, 0.0, 0.0, 1.0);
        c.radius = radius;
        return sim::apply_commands(world.world(), {c}).front();
    }
    void tick(const std::vector<Command>& cmds = {}) { world.tick(SimTick{world.ticks_simulated()}, cmds); }
    void run(int ticks) { for (int i = 0; i < ticks; ++i) tick(); }
    Vec3 pos(Entity e) const { return world.world().get<Position>(e).value; }
    bool ordered(Entity e) const { return world.world().has<Destination>(e); }
    double dist(Entity a, Entity b) const {
        const Vec3 d = pos(a) - pos(b);
        return std::sqrt(d.x * d.x + d.y * d.y);
    }
    SimWorld world;
};

double min_pair_distance(const Scene& s, const std::vector<Entity>& es) {
    double best = 1e18;
    for (std::size_t i = 0; i < es.size(); ++i)
        for (std::size_t j = i + 1; j < es.size(); ++j) best = std::min(best, s.dist(es[i], es[j]));
    return best;
}

}  // namespace

TEST_CASE("crowd: overlapping idle units push apart until they just touch", "[crowd]") {
    Scene s;
    const Entity a = s.spawn(at(10.0, 10.0));
    const Entity b = s.spawn(at(10.4, 10.0));
    s.run(120);
    CHECK(s.dist(a, b) >= 0.99);  // radii 0.5 + 0.5
    CHECK(s.dist(a, b) < 1.2);    // and no explosive overshoot
    CHECK(s.pos(a).x < s.pos(b).x);  // they kept their sides
}

TEST_CASE("crowd: units on exactly the same spot are split deterministically", "[crowd]") {
    Scene s;
    const Entity a = s.spawn(at(10.0, 10.0));
    const Entity b = s.spawn(at(10.0, 10.0));
    s.run(60);
    CHECK(s.dist(a, b) >= 0.99);
    CHECK(s.pos(a).x != s.pos(b).x);
    CHECK(s.pos(a).y == 10.0);  // split along x
}

TEST_CASE("crowd: a moving unit yields more than an idle one", "[crowd]") {
    Scene s;
    const Entity idle = s.spawn(at(10.0, 10.0));
    const Entity mover = s.spawn(at(10.5, 10.0));
    s.tick({Command::move_to(mover, at(10.5, 30.0), 0.001)});  // an order, but practically standing still
    const double idle_before = s.pos(idle).x, mover_before = s.pos(mover).x;
    s.run(5);
    const double idle_moved = std::abs(s.pos(idle).x - idle_before);
    const double mover_moved = std::abs(s.pos(mover).x - mover_before);
    CHECK(mover_moved > 2.0 * idle_moved);
    CHECK(idle_moved > 0.0);
}

TEST_CASE("crowd: units without a Radius ignore each other (legacy behaviour)", "[crowd]") {
    Scene s;
    const Entity a = s.spawn(at(10.0, 10.0), 0.0);
    const Entity b = s.spawn(at(10.0, 10.0), 0.0);
    s.run(60);
    CHECK(s.pos(a).x == 10.0);
    CHECK(s.pos(b).x == 10.0);
}

TEST_CASE("crowd: a blob of 100 units relaxes without overlap and stays deterministic", "[crowd][determinism]") {
    auto build = [] {
        auto s = std::make_unique<Scene>();
        for (int i = 0; i < 100; ++i) {
            (void)s->spawn(at(18.0 + (i % 10) * 0.3, 18.0 + (i / 10) * 0.3));
        }
        return s;
    };
    auto a = build();
    auto b = build();
    a->run(600);
    b->run(600);
    CHECK(sim::world_checksum(a->world) == sim::world_checksum(b->world));

    std::vector<Entity> all;
    a->world.world().each<Position>([&](Entity e, const Position&) { all.push_back(e); });
    REQUIRE(all.size() == 100);
    CHECK(min_pair_distance(*a, all) > 0.9);  // crowd relaxed (radii sum to 1.0)
    for (const Entity e : all) {              // and stayed on the map
        CHECK(a->pos(e).x > 0.0);
        CHECK(a->pos(e).x < 40.0);
    }
}

TEST_CASE("crowd: pushes never move a unit into a wall", "[crowd]") {
    NavGrid g(1.0, 0.0, 0.0, 40, 40);
    (void)g.block_rect(20.0, 0.0, 40.0, 40.0);  // everything x >= 20 is wall
    Scene s(NavMap::create(std::move(g), {0.5}));
    std::vector<Entity> es;
    for (int i = 0; i < 12; ++i) es.push_back(s.spawn(at(18.5 + 0.1 * (i % 3), 10.0 + 0.4 * i)));  // pressed against the wall
    s.run(300);
    for (const Entity e : es) CHECK(s.pos(e).x < 20.0);
}

TEST_CASE("crowd: a group ordered to one point arrives, gives up, and settles without stacking", "[crowd]") {
    Scene s;
    std::vector<Entity> es;
    for (int i = 0; i < 12; ++i) es.push_back(s.spawn(at(2.0 + 1.2 * i, 5.0)));
    std::vector<Command> orders;
    for (const Entity e : es) orders.push_back(Command::move_to(e, at(20.0, 20.0), 6.0));
    s.tick(orders);
    s.run(60 * 25);
    for (const Entity e : es) {
        CHECK_FALSE(s.ordered(e));  // every order ended (arrived or gave up)
        CHECK(s.dist(e, es[0]) < 8.0);
        CHECK(std::hypot(s.pos(e).x - 20.0, s.pos(e).y - 20.0) < 6.0);
    }
    s.run(600);
    CHECK(min_pair_distance(s, es) > 0.85);
    // Nothing is left half-tracked.
    for (const Entity e : es) CHECK_FALSE(s.world.world().has<MoveProgress>(e));
}

TEST_CASE("crowd: a unit whose goal is held by an idle unit stops next to it instead of pushing forever", "[crowd]") {
    Scene s;
    const Entity holder = s.spawn(at(20.0, 20.0));
    const Entity walker = s.spawn(at(10.0, 20.0));
    s.tick({Command::move_to(walker, at(20.0, 20.0), 5.0)});
    s.run(60 * 12);
    CHECK_FALSE(s.ordered(walker));
    CHECK(s.dist(walker, holder) >= 0.9);
    CHECK(s.dist(walker, holder) < 3.0);
}

TEST_CASE("crowd: a snapshot taken mid-crowd continues exactly like the original", "[crowd][replay]") {
    const auto map = open_map();
    Scene a(map);
    for (int i = 0; i < 30; ++i) {
        (void)a.spawn(at(10.0 + 0.4 * (i % 6), 10.0 + 0.4 * (i / 6)));
    }
    std::vector<Entity> es;
    a.world.world().each<Position>([&](Entity e, const Position&) { es.push_back(e); });
    std::vector<Command> orders;
    for (std::size_t i = 0; i < es.size(); i += 2) orders.push_back(Command::move_to(es[i], at(30.0, 30.0), 4.0));
    a.tick(orders);
    a.run(90);  // moves and stall windows are in flight

    core::serialization::ByteWriter w;
    sim::save_world(a.world, w);
    SimWorld b(1);
    b.set_nav_map(map);
    core::serialization::ByteReader r(w.bytes());
    sim::load_world(b, r);
    REQUIRE(sim::world_checksum(b) == sim::world_checksum(a.world));

    for (int i = 0; i < 200; ++i) {
        a.tick();
        b.tick(SimTick{b.ticks_simulated()}, {});
    }
    CHECK(sim::world_checksum(b) == sim::world_checksum(a.world));
}
