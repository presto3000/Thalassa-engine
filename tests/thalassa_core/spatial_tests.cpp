#include <algorithm>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "thalassa/core/spatial/uniform_grid.hpp"

using thalassa::core::math::Vec3;
using thalassa::core::spatial::UniformGrid;

TEST_CASE("UniformGrid query_radius finds entries within range", "[spatial]") {
    UniformGrid<int> grid(5.0);
    grid.insert(1, Vec3{0.0, 0.0, 0.0});
    grid.insert(2, Vec3{1.0, 1.0, 0.0});
    grid.insert(3, Vec3{50.0, 50.0, 0.0});   // far away, different cell entirely
    grid.insert(4, Vec3{4.9, 0.0, 0.0});     // just inside radius, likely a neighboring cell

    std::vector<int> results;
    grid.query_radius(Vec3{0.0, 0.0, 0.0}, 5.0, results);

    std::sort(results.begin(), results.end());
    REQUIRE(results == std::vector<int>{1, 2, 4});
}

TEST_CASE("UniformGrid query_radius excludes entries outside the radius even in a scanned cell", "[spatial]") {
    // Two entries in the same cell, but one just outside the query radius —
    // proves we filter by actual distance, not just cell membership.
    UniformGrid<int> grid(100.0);  // one huge cell covers both points
    grid.insert(1, Vec3{0.0, 0.0, 0.0});
    grid.insert(2, Vec3{10.0, 0.0, 0.0});

    std::vector<int> results;
    grid.query_radius(Vec3{0.0, 0.0, 0.0}, 5.0, results);

    REQUIRE(results == std::vector<int>{1});
}

TEST_CASE("UniformGrid query_aabb finds entries inside the box", "[spatial]") {
    UniformGrid<int> grid(10.0);
    grid.insert(1, Vec3{1.0, 1.0, 0.0});
    grid.insert(2, Vec3{9.0, 9.0, 0.0});
    grid.insert(3, Vec3{20.0, 20.0, 0.0});

    std::vector<int> results;
    grid.query_aabb(Vec3{0.0, 0.0, 0.0}, Vec3{10.0, 10.0, 0.0}, results);

    std::sort(results.begin(), results.end());
    REQUIRE(results == std::vector<int>{1, 2});
}

TEST_CASE("UniformGrid handles negative coordinates correctly", "[spatial]") {
    UniformGrid<int> grid(5.0);
    grid.insert(1, Vec3{-1.0, -1.0, 0.0});
    grid.insert(2, Vec3{-100.0, -100.0, 0.0});

    std::vector<int> results;
    grid.query_radius(Vec3{0.0, 0.0, 0.0}, 5.0, results);

    REQUIRE(results == std::vector<int>{1});
}

TEST_CASE("UniformGrid query result order is deterministic across repeated identical builds", "[spatial]") {
    auto build_and_query = []() {
        UniformGrid<int> grid(3.0);
        for (int i = 0; i < 200; ++i) {
            grid.insert(i, Vec3{static_cast<double>(i % 20), static_cast<double>(i / 20), 0.0});
        }
        std::vector<int> results;
        grid.query_radius(Vec3{10.0, 5.0, 0.0}, 8.0, results);
        return results;
    };

    REQUIRE(build_and_query() == build_and_query());
}

TEST_CASE("UniformGrid clear removes all entries", "[spatial]") {
    UniformGrid<int> grid(5.0);
    grid.insert(1, Vec3{0.0, 0.0, 0.0});
    REQUIRE(grid.size() == 1);
    grid.clear();
    REQUIRE(grid.size() == 0);

    std::vector<int> results;
    grid.query_radius(Vec3{0.0, 0.0, 0.0}, 100.0, results);
    REQUIRE(results.empty());
}
