#include <algorithm>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "thalassa/sim/components.hpp"
#include "thalassa/sim/sim_world.hpp"

using thalassa::core::ecs::Entity;
using thalassa::core::math::Vec3;
using thalassa::sim::SimWorld;
using thalassa::sim::components::Position;

TEST_CASE("SpatialIndex reflects post-movement positions after tick()", "[spatial][sim]") {
    SimWorld world(1);
    const Entity e = world.world().create_entity();
    world.world().emplace<Position>(e, Position{{0.0, 0.0, 0.0}});

    world.tick(thalassa::core::SimTick{0});

    std::vector<Entity> results;
    world.spatial_index().query_radius(Vec3{0.0, 0.0, 0.0}, 1.0, results);
    REQUIRE(results.size() == 1);
    REQUIRE(results[0] == e);
}

TEST_CASE("SpatialIndex query at scale finds only the nearby subset", "[spatial][sim]") {
    SimWorld world(1);
    constexpr int kCount = 2000;
    std::vector<Entity> near_origin_expected;

    for (int i = 0; i < kCount; ++i) {
        const Entity e = world.world().create_entity();
        // Half the units clustered near the origin, half far away.
        const bool near = (i % 2 == 0);
        const thalassa::core::Scalar x = near ? (world.rng().next_scalar01() * 10.0 - 5.0)
                                               : (world.rng().next_scalar01() * 10.0 + 500.0);
        const thalassa::core::Scalar y = near ? (world.rng().next_scalar01() * 10.0 - 5.0)
                                               : (world.rng().next_scalar01() * 10.0 + 500.0);
        world.world().emplace<Position>(e, Position{{x, y, 0.0}});
        if (near) {
            near_origin_expected.push_back(e);
        }
    }

    world.tick(thalassa::core::SimTick{0});

    std::vector<Entity> results;
    world.spatial_index().query_radius(Vec3{0.0, 0.0, 0.0}, 20.0, results);

    REQUIRE(results.size() == near_origin_expected.size());

    auto sort_by_index = [](const Entity& a, const Entity& b) { return a.index() < b.index(); };
    std::sort(results.begin(), results.end(), sort_by_index);
    std::sort(near_origin_expected.begin(), near_origin_expected.end(), sort_by_index);
    REQUIRE(results == near_origin_expected);
}
