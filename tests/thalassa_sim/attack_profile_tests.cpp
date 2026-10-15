#include <cstdint>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "thalassa/sim/components.hpp"
#include "thalassa/sim/replay_recording.hpp"
#include "thalassa/sim/sim_world.hpp"

using thalassa::core::SimTick;
using thalassa::core::ecs::Entity;
using thalassa::sim::Command;
using thalassa::sim::ReplayPlayer;
using thalassa::sim::ReplayRecorder;
using thalassa::sim::SimEvent;
using thalassa::sim::SimWorld;
using thalassa::sim::apply_commands;
using thalassa::sim::components::AttackState;
using thalassa::sim::components::CombatStats;
using thalassa::sim::components::Health;
using thalassa::sim::components::Position;
using thalassa::sim::components::Projectile;
using Type = SimEvent::Type;

namespace {

Entity spawn(SimWorld& w, const Command& c) { return apply_commands(w.world(), {c})[0]; }

std::size_t projectile_count(SimWorld& w) {
    std::size_t n = 0;
    w.world().each<Projectile>([&n](Entity, const Projectile&) { ++n; });
    return n;
}

void run(SimWorld& w, std::uint64_t ticks, std::vector<SimEvent>* out = nullptr) {
    for (std::uint64_t i = 0; i < ticks; ++i) {
        w.tick(SimTick{w.ticks_simulated()});
        if (out != nullptr) {
            auto ev = w.drain_events();
            out->insert(out->end(), ev.begin(), ev.end());
        }
    }
}

const SimEvent* find(const std::vector<SimEvent>& ev, Type t) {
    for (const auto& e : ev) if (e.type == t) return &e;
    return nullptr;
}

bool same(const SimEvent& a, const SimEvent& b) {
    return a.type == b.type && a.tick == b.tick && a.source == b.source && a.target == b.target &&
           a.projectile == b.projectile && a.position == b.position && a.value == b.value && a.value2 == b.value2;
}

}  // namespace

TEST_CASE("events are off by default and cost nothing", "[attack]") {
    SimWorld w(1);
    spawn(w, Command::spawn_unit({0, 0, 0}, {}, 0, 100, 10, 5, 1.0).as_melee(0.2));
    spawn(w, Command::spawn_unit({1, 0, 0}, {}, 1, 100, 0, 0, 1.0));
    run(w, 60);
    REQUIRE(w.drain_events().empty());
}

TEST_CASE("melee: damage lands after the windup, with no projectile entity", "[attack]") {
    SimWorld w(1);
    w.set_events_enabled(true);
    const Entity hero = spawn(w, Command::spawn_unit({0, 0, 0}, {}, 0, 100, 20, 2.0, 1.0).as_melee(0.25));
    const Entity dummy = spawn(w, Command::spawn_unit({1.5, 0, 0}, {}, 1, 100, 0, 0, 1.0));

    std::vector<SimEvent> ev;
    run(w, 10, &ev);  // 10 ticks = 0.167 s < windup 0.25 s
    REQUIRE(w.world().get<Health>(dummy).current == 100.0);
    REQUIRE(w.world().get<AttackState>(hero).phase == AttackState::kWindup);
    const SimEvent* started = find(ev, Type::AttackStarted);
    REQUIRE(started != nullptr);
    REQUIRE(started->value == 0.25);   // windup: when to release
    REQUIRE(started->value2 == 1.0);   // whole swing (cooldown): backswing = value2 - value
    REQUIRE(find(ev, Type::MeleeHit) == nullptr);

    run(w, 10, &ev);  // 20 ticks total > 15
    REQUIRE(w.world().get<Health>(dummy).current == 80.0);
    const SimEvent* hit = find(ev, Type::MeleeHit);
    REQUIRE(hit != nullptr);
    REQUIRE(hit->source == hero);
    REQUIRE(hit->target == dummy);
    REQUIRE(hit->value == 20.0);
    REQUIRE(hit->tick >= 14);
    REQUIRE(hit->tick <= 16);
    REQUIRE(projectile_count(w) == 0);
    REQUIRE(find(ev, Type::ProjectileLaunched) == nullptr);
}

TEST_CASE("melee: cooldown counts from attack start, so hits repeat every cooldown", "[attack]") {
    SimWorld w(1);
    w.set_events_enabled(true);
    spawn(w, Command::spawn_unit({0, 0, 0}, {}, 0, 100, 10, 2.0, 0.5).as_melee(0.1));  // 30-tick cooldown
    const Entity dummy = spawn(w, Command::spawn_unit({1, 0, 0}, {}, 1, 1000, 0, 0, 1.0));
    run(w, 95);  // starts at ticks 0, 30, 60, 90 -> 4 swings, 3 or 4 landed by tick 95
    const double hp = w.world().get<Health>(dummy).current;
    REQUIRE((hp == 970.0 || hp == 960.0));
}

TEST_CASE("ranged: projectile is released after the windup at the hero's own speed", "[attack]") {
    SimWorld w(1);
    w.set_events_enabled(true);
    const Entity hero =
        spawn(w, Command::spawn_unit({0, 0, 0}, {}, 0, 100, 15, 30.0, 1.0).as_ranged(0.3, 20.0));
    const Entity dummy = spawn(w, Command::spawn_unit({20, 0, 0}, {}, 1, 100, 0, 0, 1.0));

    std::vector<SimEvent> ev;
    run(w, 10, &ev);
    REQUIRE(projectile_count(w) == 0);  // still winding up
    REQUIRE(find(ev, Type::AttackStarted) != nullptr);

    run(w, 15, &ev);  // windup (0.3 s = 18 ticks) done
    const SimEvent* launched = find(ev, Type::ProjectileLaunched);
    REQUIRE(launched != nullptr);
    REQUIRE(launched->source == hero);
    REQUIRE(launched->target == dummy);
    REQUIRE(launched->value == 15.0);

    // 20 units at 20 u/s = 1 s of flight.
    run(w, 90, &ev);
    REQUIRE(w.world().get<Health>(dummy).current == 85.0);
    const SimEvent* hit = find(ev, Type::ProjectileHit);
    REQUIRE(hit != nullptr);
    REQUIRE(hit->target == dummy);
    REQUIRE(hit->projectile == launched->projectile);
}

TEST_CASE("windup is cancelled (and cooldown refunded) when the target leaves range", "[attack]") {
    SimWorld w(1);
    w.set_events_enabled(true);
    const Entity hero = spawn(w, Command::spawn_unit({0, 0, 0}, {}, 0, 100, 10, 2.0, 5.0).as_melee(0.5));
    const Entity dummy = spawn(w, Command::spawn_unit({1, 0, 0}, {}, 1, 100, 0, 0, 1.0));

    std::vector<SimEvent> ev;
    run(w, 5, &ev);
    REQUIRE(w.world().get<AttackState>(hero).phase == AttackState::kWindup);

    w.world().get<Position>(dummy).value = {50, 0, 0};  // runs away mid-windup
    run(w, 2, &ev);
    REQUIRE(find(ev, Type::AttackCancelled) != nullptr);
    REQUIRE(w.world().get<AttackState>(hero).phase == AttackState::kIdle);
    REQUIRE(w.world().get<CombatStats>(hero).cooldown_remaining_seconds == 0.0);
    REQUIRE(w.world().get<Health>(dummy).current == 100.0);

    // Comes back: the hero starts a fresh attack immediately (cooldown refunded).
    w.world().get<Position>(dummy).value = {1, 0, 0};
    ev.clear();
    run(w, 2, &ev);
    REQUIRE(find(ev, Type::AttackStarted) != nullptr);
}

TEST_CASE("units without a profile keep the legacy instant projectile", "[attack]") {
    SimWorld w(1);
    w.set_events_enabled(true);
    spawn(w, Command::spawn_unit({0, 0, 0}, {}, 0, 100, 10, 20, 1.0));  // no .as_*()
    spawn(w, Command::spawn_unit({10, 0, 0}, {}, 1, 100, 0, 0, 1.0));
    std::vector<SimEvent> ev;
    run(w, 2, &ev);
    REQUIRE(find(ev, Type::ProjectileLaunched) != nullptr);
    REQUIRE(find(ev, Type::AttackStarted) == nullptr);
}

TEST_CASE("UnitDied is reported with the death position", "[attack]") {
    SimWorld w(1);
    w.set_events_enabled(true);
    spawn(w, Command::spawn_unit({0, 0, 0}, {}, 0, 100, 500, 3.0, 1.0).as_melee(0.0));
    const Entity victim = spawn(w, Command::spawn_unit({1, 2, 0}, {}, 1, 100, 0, 0, 1.0));
    std::vector<SimEvent> ev;
    run(w, 3, &ev);
    const SimEvent* died = find(ev, Type::UnitDied);
    REQUIRE(died != nullptr);
    REQUIRE(died->target == victim);
    REQUIRE(died->position == thalassa::core::math::Vec3{1, 2, 0});
    REQUIRE_FALSE(w.world().is_alive(victim));
}

TEST_CASE("mixed melee+ranged fight: replay reproduces state AND the exact event stream", "[attack][replay]") {
    ReplayRecorder rec(99, 1.0 / 60.0, 30);
    rec.sim().set_events_enabled(true);

    std::vector<Command> spawns;
    for (int i = 0; i < 6; ++i) {
        const double y = i * 2.0;
        Command a = Command::spawn_unit({-6, y, 0}, {}, 0, 120, 14, 2.0, 0.7);
        Command b = Command::spawn_unit({6, y, 0}, {}, 1, 90, 9, 25.0, 0.8);
        spawns.push_back(i % 2 == 0 ? a.as_melee(0.25) : a.as_ranged(0.2, 30.0));
        spawns.push_back(i % 2 == 0 ? b.as_ranged(0.3, 25.0) : b.as_melee(0.15));
    }
    rec.step(spawns);

    std::vector<SimEvent> recorded;
    auto collect = [&] {
        auto e = rec.sim().drain_events();
        recorded.insert(recorded.end(), e.begin(), e.end());
    };
    collect();
    for (int i = 0; i < 900; ++i) {
        rec.step();
        collect();
    }
    const auto recording = rec.finish();
    REQUIRE(find(recorded, Type::MeleeHit) != nullptr);
    REQUIRE(find(recorded, Type::ProjectileHit) != nullptr);
    REQUIRE(find(recorded, Type::UnitDied) != nullptr);

    ReplayPlayer p(recording);
    std::vector<SimEvent> replayed;
    while (p.step_tick()) {
        auto e = p.drain_events();
        replayed.insert(replayed.end(), e.begin(), e.end());
    }
    REQUIRE_FALSE(p.desynced());
    REQUIRE(p.final_checksum_matches());
    REQUIRE(replayed.size() == recorded.size());
    for (std::size_t i = 0; i < recorded.size(); ++i) {
        REQUIRE(same(recorded[i], replayed[i]));
    }
}

TEST_CASE("attack profile survives replay file serialization", "[attack][replay]") {
    ReplayRecorder rec(5, 1.0 / 60.0, 0);
    (void)rec.submit(Command::spawn_unit({0, 0, 0}, {}, 0, 100, 10, 2.0, 1.0).as_melee(0.4));
    (void)rec.submit(Command::spawn_unit({4, 0, 0}, {}, 1, 100, 10, 20.0, 1.0).as_ranged(0.35, 18.0));
    for (int i = 0; i < 120; ++i) rec.step();
    const auto r = rec.finish();

    thalassa::core::serialization::ByteWriter w;
    r.serialize(w);
    thalassa::sim::ReplayRecording loaded;
    thalassa::core::serialization::ByteReader reader(w.bytes());
    REQUIRE(loaded.deserialize(reader));

    const auto* cmds = loaded.commands.commands_at(0);
    REQUIRE(cmds != nullptr);
    REQUIRE(cmds->size() == 2);
    REQUIRE((*cmds)[0].attack_kind == 0);
    REQUIRE((*cmds)[0].attack_windup_seconds == 0.4);
    REQUIRE((*cmds)[1].attack_kind == 1);
    REQUIRE((*cmds)[1].projectile_speed == 18.0);

    ReplayPlayer p(loaded);
    while (p.step_tick()) {}
    REQUIRE(p.final_checksum_matches());
}
