#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "thalassa/core/serialization/byte_stream.hpp"
#include "thalassa/sim/commands.hpp"
#include "thalassa/sim/components.hpp"
#include "thalassa/sim/group_move.hpp"
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
using sim::components::Chase;
using sim::components::Destination;
using sim::components::Health;
using sim::components::Position;
using sim::nav::NavGrid;
using sim::nav::NavMap;

namespace {

Vec3 at(double x, double y) { return Vec3{x, y, 0.0}; }

std::shared_ptr<const NavMap> open_map() { return NavMap::create(NavGrid(1.0, 0.0, 0.0, 60, 60), {0.5}); }

// Wall at x in [30,31) from y=0 to 40; open above.
std::shared_ptr<const NavMap> walled_map() {
    NavGrid g(1.0, 0.0, 0.0, 60, 60);
    (void)g.block_rect(30.0, 0.0, 31.0, 40.0);
    return NavMap::create(std::move(g), {0.5});
}

struct Scene {
    explicit Scene(std::shared_ptr<const NavMap> map = nullptr) : world(1) {
        if (map) world.set_nav_map(std::move(map));
    }
    // team 0 = attackers, team 1 = defenders; melee by default.
    Entity spawn(Vec3 p, int team, double range = 3.0, bool ranged = false, double hp = 1.0e6, double damage = 10.0) {
        Command c = Command::spawn_unit(p, core::PlayerId{static_cast<std::uint32_t>(team + 1)}, team, hp, damage, range, 0.5);
        c = ranged ? c.as_ranged(0.1, 40.0) : c.as_melee(0.1);
        return sim::apply_commands(world.world(), {c}).front();
    }
    void tick(const std::vector<Command>& cmds = {}) { world.tick(SimTick{world.ticks_simulated()}, cmds); }
    void run(int ticks) { for (int i = 0; i < ticks; ++i) tick(); }
    Vec3 pos(Entity e) const { return world.world().get<Position>(e).value; }
    double hp(Entity e) const { return world.world().is_alive(e) ? world.world().get<Health>(e).current : 0.0; }
    double dist(Entity a, Entity b) const {
        const Vec3 d = pos(a) - pos(b);
        return std::sqrt(d.x * d.x + d.y * d.y);
    }
    bool chasing(Entity e) const { return world.world().is_alive(e) && world.world().has<Chase>(e); }
    bool walking(Entity e) const { return world.world().is_alive(e) && world.world().has<Destination>(e); }
    SimWorld world;
};

}  // namespace

TEST_CASE("chase: a melee unit walks up to a still target and fights it", "[chase]") {
    Scene s;
    const Entity hunter = s.spawn(at(10.0, 10.0), 0);
    const Entity prey = s.spawn(at(40.0, 10.0), 1, 3.0, false, 1000.0);
    s.tick({Command::attack_target(hunter, prey, 6.0)});
    s.run(60 * 8);
    CHECK(s.dist(hunter, prey) <= 3.0);
    CHECK(s.dist(hunter, prey) > 2.0);  // stopped just inside range, not on top of it
    CHECK_FALSE(s.walking(hunter));
    CHECK(s.hp(prey) < 1000.0);
}

TEST_CASE("chase: a ranged unit stops at its range, not at the target", "[chase]") {
    Scene s;
    const Entity archer = s.spawn(at(10.0, 10.0), 0, 15.0, true);
    const Entity prey = s.spawn(at(50.0, 10.0), 1, 3.0, false, 1000.0);
    s.tick({Command::attack_target(archer, prey, 6.0)});
    s.run(60 * 10);
    CHECK(s.dist(archer, prey) <= 15.0);
    CHECK(s.dist(archer, prey) > 12.0);
    CHECK(s.hp(prey) < 1000.0);
}

TEST_CASE("chase: a moving target is followed and caught", "[chase]") {
    Scene s(open_map());
    const Entity hunter = s.spawn(at(5.0, 20.0), 0);
    const Entity prey = s.spawn(at(15.0, 20.0), 1, 3.0, false, 1000.0);
    s.tick({Command::attack_target(hunter, prey, 6.0), Command::move_to(prey, at(55.0, 40.0), 3.0)});
    s.run(60 * 25);
    CHECK(s.dist(hunter, prey) <= 3.0);
    CHECK(s.hp(prey) < 1000.0);
    // The prey got far from where the hunter began, so the chase really tracked it.
    CHECK(s.pos(hunter).x > 20.0);
}

TEST_CASE("chase: the route goes around walls and re-planning stays bounded", "[chase]") {
    Scene s(walled_map());
    const Entity hunter = s.spawn(at(10.0, 10.0), 0);
    const Entity prey = s.spawn(at(50.0, 10.0), 1, 3.0, false, 1000.0);
    s.tick({Command::attack_target(hunter, prey, 8.0), Command::move_to(prey, at(50.0, 30.0), 2.0)});
    double max_y = 0.0;
    for (int i = 0; i < 60 * 40; ++i) {
        s.tick();
        max_y = std::max(max_y, s.pos(hunter).y);
    }
    CHECK(max_y > 39.0);  // went over the wall's top
    CHECK(s.dist(hunter, prey) <= 3.0);
    // Searches: one per replan, at most one per kMinReplanTicks while chasing (plus the order itself).
    CHECK(s.world.path_planner()->searches() <= 60 * 40 / Chase::kMinReplanTicks + 5);
}

TEST_CASE("chase: the order ends when the target dies, and a MoveTo cancels it", "[chase]") {
    Scene s;
    const Entity hunter = s.spawn(at(10.0, 10.0), 0, 3.0, false, 100.0, 200.0);  // one-shot kills
    const Entity prey = s.spawn(at(14.0, 10.0), 1, 3.0, false, 50.0);
    s.tick({Command::attack_target(hunter, prey, 6.0)});
    s.run(60 * 4);
    CHECK_FALSE(s.world.world().is_alive(prey));
    CHECK_FALSE(s.chasing(hunter));
    CHECK_FALSE(s.walking(hunter));

    const Entity other = s.spawn(at(30.0, 30.0), 1, 3.0, false, 1000.0);
    s.tick({Command::attack_target(hunter, other, 6.0)});
    CHECK(s.chasing(hunter));
    s.tick({Command::move_to(hunter, at(10.0, 10.0), 6.0)});
    CHECK_FALSE(s.chasing(hunter));
    CHECK(s.walking(hunter));
}

TEST_CASE("chase: an attack order is ignored for dead or invalid participants", "[chase]") {
    Scene s;
    const Entity a = s.spawn(at(10.0, 10.0), 0);
    s.tick({Command::attack_target(a, a, 5.0)});                  // itself
    CHECK_FALSE(s.chasing(a));
    s.tick({Command::attack_target(a, Entity{99, 7}, 5.0)});      // no such entity
    CHECK_FALSE(s.chasing(a));
}

TEST_CASE("chase: the chased enemy is preferred over a nearer one in range", "[chase]") {
    Scene s;
    const Entity hunter = s.spawn(at(10.0, 10.0), 0, 15.0, true, 100.0, 5.0);
    const Entity near_enemy = s.spawn(at(10.0, 13.5), 1, 3.0, false, 1000.0);  // nearer, but off the firing line
    const Entity wanted = s.spawn(at(20.0, 10.0), 1, 3.0, false, 1000.0);
    s.tick({Command::attack_target(hunter, wanted, 6.0)});
    s.run(60 * 3);
    CHECK(s.hp(wanted) < 1000.0);
    CHECK(s.hp(near_enemy) == 1000.0);  // never shot while the wanted target is in range
}

TEST_CASE("chase: an unreachable target gives a partial route instead of a crash", "[chase]") {
    NavGrid g(1.0, 0.0, 0.0, 60, 60);
    (void)g.block_rect(30.0, 0.0, 31.0, 60.0);  // full wall
    Scene s(NavMap::create(std::move(g), {0.5}));
    const Entity hunter = s.spawn(at(10.0, 10.0), 0);
    const Entity prey = s.spawn(at(50.0, 10.0), 1, 3.0, false, 1000.0);
    s.tick({Command::attack_target(hunter, prey, 6.0)});
    s.run(60 * 15);
    CHECK(s.pos(hunter).x < 30.0);
    CHECK(s.pos(hunter).x > 25.0);  // got as close as the wall allows
    CHECK(s.chasing(hunter));       // still hunting
}

TEST_CASE("chase: a snapshot taken mid-chase continues exactly like the original", "[chase][replay]") {
    const auto map = walled_map();
    Scene a(map);
    const Entity hunter = a.spawn(at(10.0, 10.0), 0);
    const Entity prey = a.spawn(at(50.0, 10.0), 1, 3.0, false, 1000.0);
    a.tick({Command::attack_target(hunter, prey, 8.0), Command::move_to(prey, at(50.0, 30.0), 2.0)});
    a.run(200);

    core::serialization::ByteWriter w;
    sim::save_world(a.world, w);
    SimWorld b(1);
    b.set_nav_map(map);
    core::serialization::ByteReader r(w.bytes());
    sim::load_world(b, r);
    REQUIRE(sim::world_checksum(b) == sim::world_checksum(a.world));
    for (int i = 0; i < 600; ++i) {
        a.tick();
        b.tick(SimTick{b.ticks_simulated()}, {});
    }
    CHECK(sim::world_checksum(b) == sim::world_checksum(a.world));
}

TEST_CASE("chase: replays with attack orders play back bit-exact", "[chase][replay]") {
    sim::ReplayRecorder rec(3, 1.0 / 60.0, 30);
    rec.set_nav_map(walled_map());
    Command h = Command::spawn_unit(at(10.0, 10.0), core::PlayerId{1}, 0, 100.0, 10.0, 3.0, 0.5).as_melee(0.1).with_radius(0.4);
    Command p = Command::spawn_unit(at(50.0, 10.0), core::PlayerId{2}, 1, 1000.0, 10.0, 3.0, 0.5).as_melee(0.1).with_radius(0.4);
    const Entity hunter = rec.submit(h).front();
    const Entity prey = rec.submit(p).front();
    rec.step({Command::attack_target(hunter, prey, 8.0)});
    for (int i = 0; i < 60 * 30; ++i) rec.step();
    const sim::ReplayRecording recording = rec.finish();

    core::serialization::ByteWriter w;
    recording.serialize(w);
    sim::ReplayRecording loaded;
    core::serialization::ByteReader r(w.bytes());
    REQUIRE(loaded.deserialize(r));
    sim::ReplayPlayer player(loaded);
    while (player.step_tick()) {}
    CHECK_FALSE(player.desynced());
    CHECK(player.final_checksum_matches());
}

TEST_CASE("group move: spread slots are distinct and at least `spacing` apart", "[group]") {
    for (const std::size_t n : {0u, 1u, 2u, 7u, 8u, 19u, 20u, 61u, 150u}) {
        const auto slots = sim::spread_slots(at(30.0, 30.0), n, 1.5);
        REQUIRE(slots.size() == n);
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = i + 1; j < n; ++j) {
                const Vec3 d = slots[i] - slots[j];
                CHECK(std::sqrt(d.x * d.x + d.y * d.y) >= 1.5 - 1e-9);
            }
    }
    CHECK(sim::spread_slots(at(5.0, 6.0), 1, 2.0)[0].x == 5.0);  // the first slot is the center
}

TEST_CASE("group move: each unit gets its own slot, nearest units get the inner ones", "[group]") {
    Scene s;
    std::vector<Entity> es;
    for (int i = 0; i < 10; ++i) es.push_back(s.spawn(at(5.0 + 2.0 * i, 10.0), 0));
    const Vec3 center = at(40.0, 10.0);
    const auto cmds = sim::plan_group_move(s.world.world(), es, center, 5.0, 1.5);
    REQUIRE(cmds.size() == 10);
    std::set<std::pair<double, double>> goals;
    for (const Command& c : cmds) goals.insert({c.position.x, c.position.y});
    CHECK(goals.size() == 10);
    // es[9] is closest to the center: it gets the center slot.
    const auto it = std::find_if(cmds.begin(), cmds.end(), [&](const Command& c) { return c.entity == es[9]; });
    REQUIRE(it != cmds.end());
    CHECK(it->position.x == center.x);
    // Same input -> same commands.
    const auto again = sim::plan_group_move(s.world.world(), es, center, 5.0, 1.5);
    for (std::size_t i = 0; i < cmds.size(); ++i) {
        CHECK(cmds[i].entity == again[i].entity);
        CHECK(cmds[i].position.x == again[i].position.x);
        CHECK(cmds[i].position.y == again[i].position.y);
    }
}

TEST_CASE("group move: a spread group arrives with every order completed and without stacking", "[group][crowd]") {
    Scene s(open_map());
    std::vector<Entity> es;
    for (int i = 0; i < 20; ++i) {
        Command c = Command::spawn_unit(at(5.0 + 1.2 * (i % 5), 5.0 + 1.2 * (i / 5)), core::PlayerId{1}, 0, 100.0, 0.0, 0.0, 1.0);
        c.radius = 0.5;
        es.push_back(sim::apply_commands(s.world.world(), {c}).front());
    }
    s.tick(sim::plan_group_move(s.world.world(), es, at(40.0, 40.0), 6.0, 1.2));
    s.run(60 * 20);
    double nearest = 1e18, farthest = 0.0;
    for (std::size_t i = 0; i < es.size(); ++i) {
        CHECK_FALSE(s.walking(es[i]));
        farthest = std::max(farthest, std::hypot(s.pos(es[i]).x - 40.0, s.pos(es[i]).y - 40.0));
        for (std::size_t j = i + 1; j < es.size(); ++j) nearest = std::min(nearest, s.dist(es[i], es[j]));
    }
    CHECK(nearest > 0.9);
    CHECK(farthest < 6.0);  // 20 units fit within 2 hex rings of spacing 1.2
}
