#include <catch2/catch_test_macros.hpp>

#include "thalassa/core/ecs/world.hpp"

using thalassa::core::ecs::Entity;
using thalassa::core::ecs::World;

namespace {
struct Position { double x = 0, y = 0; };
struct Health { int hp = 100; };
}  // namespace

TEST_CASE("World creates and destroys entities with generation safety", "[ecs]") {
    World world;

    const Entity e1 = world.create_entity();
    REQUIRE(world.is_alive(e1));
    REQUIRE(world.alive_count() == 1);

    world.destroy_entity(e1);
    REQUIRE_FALSE(world.is_alive(e1));
    REQUIRE(world.alive_count() == 0);

    // Recycled slot must have a different generation, so the old handle
    // stays invalid even though the index is reused.
    const Entity e2 = world.create_entity();
    REQUIRE(e2.index() == e1.index());
    REQUIRE(e2.generation() != e1.generation());
    REQUIRE(world.is_alive(e2));
    REQUIRE_FALSE(world.is_alive(e1));
}

TEST_CASE("Component pools store and retrieve values", "[ecs]") {
    World world;
    const Entity e = world.create_entity();

    world.emplace<Position>(e, Position{1.0, 2.0});
    REQUIRE(world.has<Position>(e));
    REQUIRE(world.get<Position>(e).x == 1.0);
    REQUIRE(world.get<Position>(e).y == 2.0);

    world.remove<Position>(e);
    REQUIRE_FALSE(world.has<Position>(e));
}

TEST_CASE("Destroying an entity removes it from all component pools", "[ecs]") {
    World world;
    const Entity e = world.create_entity();
    world.emplace<Position>(e, Position{1.0, 2.0});
    world.emplace<Health>(e, Health{50});

    world.destroy_entity(e);

    REQUIRE(world.component_count<Position>() == 0);
    REQUIRE(world.component_count<Health>() == 0);
}

TEST_CASE("each<T> iterates in insertion order", "[ecs]") {
    World world;
    std::vector<Entity> entities;
    for (int i = 0; i < 10; ++i) {
        const Entity e = world.create_entity();
        world.emplace<Position>(e, Position{static_cast<double>(i), 0.0});
        entities.push_back(e);
    }

    std::vector<double> observed;
    world.each<Position>([&observed](Entity, const Position& p) { observed.push_back(p.x); });

    REQUIRE(observed.size() == 10);
    for (int i = 0; i < 10; ++i) {
        REQUIRE(observed[static_cast<std::size_t>(i)] == static_cast<double>(i));
    }
}

TEST_CASE("each<T> iteration order is stable after swap-and-pop erase", "[ecs]") {
    World world;
    std::vector<Entity> entities;
    for (int i = 0; i < 5; ++i) {
        const Entity e = world.create_entity();
        world.emplace<Position>(e, Position{static_cast<double>(i), 0.0});
        entities.push_back(e);
    }

    // Remove the middle entity's component; the last one takes its slot.
    world.remove<Position>(entities[2]);

    std::vector<double> observed;
    world.each<Position>([&observed](Entity, const Position& p) { observed.push_back(p.x); });

    REQUIRE(observed.size() == 4);
    // Expected: [0, 1, 3, 4] -> after swap-and-pop of index 2, last (4)
    // moves into slot 2: [0, 1, 4, 3].
    REQUIRE(observed == std::vector<double>{0.0, 1.0, 4.0, 3.0});
}

TEST_CASE("each<T1,T2> only visits entities with both components", "[ecs]") {
    World world;

    const Entity a = world.create_entity();
    const Entity b = world.create_entity();
    const Entity c = world.create_entity();

    world.emplace<Position>(a, Position{1.0, 0.0});
    world.emplace<Health>(a, Health{10});

    world.emplace<Position>(b, Position{2.0, 0.0});
    // b has no Health.

    world.emplace<Health>(c, Health{30});
    // c has no Position.

    int visited = 0;
    world.each<Position, Health>([&visited, a](Entity e, Position&, Health&) {
        REQUIRE(e == a);
        ++visited;
    });

    REQUIRE(visited == 1);
}

TEST_CASE("Determinism: same operations produce same iteration results across two worlds", "[ecs]") {
    auto build = []() {
        World world;
        std::vector<double> observed;
        for (int i = 0; i < 100; ++i) {
            const Entity e = world.create_entity();
            world.emplace<Position>(e, Position{static_cast<double>(i) * 1.5, 0.0});
            if (i % 3 == 0) {
                world.emplace<Health>(e, Health{i});
            }
        }
        world.each<Position, Health>([&observed](Entity, Position& p, Health&) { observed.push_back(p.x); });
        return observed;
    };

    REQUIRE(build() == build());
}
