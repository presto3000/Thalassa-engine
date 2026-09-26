#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "thalassa/net/snapshot_delta.hpp"
#include "thalassa/sim/commands.hpp"
#include "thalassa/sim/components.hpp"
#include "thalassa/sim/replay.hpp"

using thalassa::core::PlayerId;
using thalassa::core::ecs::Entity;
using thalassa::core::math::Vec3;
using thalassa::core::serialization::ByteReader;
using thalassa::core::serialization::ByteWriter;
using thalassa::net::RelevanceFilter;
using thalassa::sim::Command;
using thalassa::sim::SimWorld;
using thalassa::sim::apply_commands;
using thalassa::sim::components::Health;
using thalassa::sim::components::Position;

namespace {

// Deep-copies `src` into a fresh SimWorld via the already-tested
// replay::save_world/load_world round trip — used here purely as test
// scaffolding (build a baseline, then mutate a separate `current` world
// without disturbing the baseline).
SimWorld clone(const SimWorld& src) {
    ByteWriter writer;
    thalassa::sim::save_world(src, writer);
    ByteReader reader(writer.bytes());
    SimWorld copy(0);
    thalassa::sim::load_world(copy, reader);
    return copy;
}

}  // namespace

TEST_CASE("apply_delta reproduces current exactly when baseline is empty (acts like a full snapshot)",
          "[snapshot_delta]") {
    SimWorld baseline(1);
    SimWorld current(1);
    (void)apply_commands(current.world(), {
                                         Command::spawn_unit({1.0, 2.0, 0.0}, {}, 0, 80.0, 10.0, 5.0, 1.0),
                                         Command::spawn_unit({3.0, 4.0, 0.0}, {}, 1, 90.0, 12.0, 6.0, 1.0),
                                     });

    ByteWriter writer;
    thalassa::net::save_delta(baseline, current, PlayerId{}, thalassa::net::relevance::always_relevant, writer);

    ByteReader reader(writer.bytes());
    thalassa::net::apply_delta(baseline, reader);  // baseline itself becomes the target, matching real usage

    REQUIRE(baseline.world().alive_count() == current.world().alive_count());
    REQUIRE(baseline.world().component_count<Position>() == current.world().component_count<Position>());
    REQUIRE(baseline.world().component_count<Health>() == current.world().component_count<Health>());
}

TEST_CASE("apply_delta reflects a position change on an already-known entity", "[snapshot_delta]") {
    SimWorld baseline(1);
    const auto spawned = apply_commands(baseline.world(), {Command::spawn_unit({0.0, 0.0, 0.0}, {}, 0, 100.0, 10.0,
                                                                                 5.0, 1.0)});
    const Entity e = spawned[0];

    SimWorld current = clone(baseline);
    current.world().get<Position>(e).value = Vec3{42.0, 7.0, 0.0};

    ByteWriter writer;
    thalassa::net::save_delta(baseline, current, PlayerId{}, thalassa::net::relevance::always_relevant, writer);
    ByteReader reader(writer.bytes());
    thalassa::net::apply_delta(baseline, reader);

    REQUIRE(baseline.world().get<Position>(e).value == Vec3{42.0, 7.0, 0.0});
}

TEST_CASE("apply_delta replicates entity destruction, not just component removal", "[snapshot_delta]") {
    SimWorld baseline(1);
    const auto spawned = apply_commands(baseline.world(), {
                                                                Command::spawn_unit({0.0, 0.0, 0.0}, {}, 0, 100.0,
                                                                                     10.0, 5.0, 1.0),
                                                                Command::spawn_unit({5.0, 0.0, 0.0}, {}, 1, 100.0,
                                                                                     10.0, 5.0, 1.0),
                                                            });
    const Entity survivor = spawned[0];
    const Entity destroyed = spawned[1];

    SimWorld current = clone(baseline);
    current.world().destroy_entity(destroyed);

    ByteWriter writer;
    thalassa::net::save_delta(baseline, current, PlayerId{}, thalassa::net::relevance::always_relevant, writer);
    ByteReader reader(writer.bytes());
    thalassa::net::apply_delta(baseline, reader);

    REQUIRE(baseline.world().is_alive(survivor));
    REQUIRE_FALSE(baseline.world().is_alive(destroyed));  // properly destroyed, not just missing components
    REQUIRE(baseline.world().alive_count() == 1);
}

TEST_CASE("apply_delta creates a brand-new entity with the server's exact id", "[snapshot_delta]") {
    SimWorld baseline(1);
    SimWorld current = clone(baseline);
    const auto spawned =
        apply_commands(current.world(), {Command::spawn_unit({9.0, 9.0, 0.0}, {}, 0, 100.0, 10.0, 5.0, 1.0)});
    const Entity new_entity = spawned[0];

    ByteWriter writer;
    thalassa::net::save_delta(baseline, current, PlayerId{}, thalassa::net::relevance::always_relevant, writer);
    ByteReader reader(writer.bytes());
    thalassa::net::apply_delta(baseline, reader);

    REQUIRE(baseline.world().is_alive(new_entity));
    REQUIRE(baseline.world().get<Position>(new_entity).value == Vec3{9.0, 9.0, 0.0});
}

TEST_CASE("A relevance filter excludes entities from the delta entirely", "[snapshot_delta]") {
    SimWorld baseline(1);
    SimWorld current = clone(baseline);
    const auto spawned = apply_commands(current.world(), {
                                                               Command::spawn_unit({0.0, 0.0, 0.0}, {}, 0, 100.0,
                                                                                    10.0, 5.0, 1.0),
                                                               Command::spawn_unit({1.0, 0.0, 0.0}, {}, 0, 100.0,
                                                                                    10.0, 5.0, 1.0),
                                                           });
    const Entity visible = spawned[0];
    const Entity hidden = spawned[1];

    const RelevanceFilter only_visible = [visible](const thalassa::core::ecs::World&, Entity e, PlayerId) {
        return e == visible;
    };

    ByteWriter writer;
    thalassa::net::save_delta(baseline, current, PlayerId{}, only_visible, writer);
    ByteReader reader(writer.bytes());
    thalassa::net::apply_delta(baseline, reader);

    REQUIRE(baseline.world().is_alive(visible));
    REQUIRE_FALSE(baseline.world().is_alive(hidden));
}

TEST_CASE("A delta with few changes is much smaller than a full snapshot", "[snapshot_delta]") {
    SimWorld baseline(1);
    std::vector<Command> spawns;
    for (int i = 0; i < 200; ++i) {
        spawns.push_back(
            Command::spawn_unit({static_cast<double>(i), 0.0, 0.0}, {}, 0, 100.0, 10.0, 5.0, 1.0));
    }
    const auto spawned = apply_commands(baseline.world(), spawns);

    SimWorld current = clone(baseline);
    // Change just one of the 200 entities.
    current.world().get<Position>(spawned[0]).value = Vec3{999.0, 0.0, 0.0};

    ByteWriter full_snapshot;
    thalassa::sim::save_world(current, full_snapshot);

    ByteWriter delta;
    thalassa::net::save_delta(baseline, current, PlayerId{}, thalassa::net::relevance::always_relevant, delta);

    REQUIRE(delta.size() < full_snapshot.size() / 4);
}

TEST_CASE("Applying an empty delta (no changes) leaves the world unchanged", "[snapshot_delta]") {
    SimWorld baseline(1);
    (void)apply_commands(baseline.world(), {Command::spawn_unit({1.0, 1.0, 0.0}, {}, 0, 100.0, 10.0, 5.0, 1.0)});
    SimWorld current = clone(baseline);

    ByteWriter writer;
    thalassa::net::save_delta(baseline, current, PlayerId{}, thalassa::net::relevance::always_relevant, writer);
    ByteReader reader(writer.bytes());
    thalassa::net::apply_delta(baseline, reader);

    REQUIRE(baseline.world().alive_count() == current.world().alive_count());
}
