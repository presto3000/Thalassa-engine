#include <cstdint>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "thalassa/core/ecs/component_pool.hpp"
#include "thalassa/core/serialization/byte_stream.hpp"

using thalassa::core::ecs::ComponentPool;
using thalassa::core::ecs::Entity;
using thalassa::core::serialization::ByteReader;
using thalassa::core::serialization::ByteWriter;

namespace {
struct Point {
    double x = 0;
    double y = 0;
    bool operator==(const Point&) const = default;
};
}  // namespace

TEST_CASE("ByteWriter/ByteReader round-trip primitives", "[serialization]") {
    ByteWriter writer;
    writer.write_u32(0xDEADBEEF);
    writer.write_u64(0x1122334455667788ULL);
    writer.write_pod<double>(3.14159265358979);

    ByteReader reader(writer.bytes());
    REQUIRE(reader.read_u32() == 0xDEADBEEF);
    REQUIRE(reader.read_u64() == 0x1122334455667788ULL);
    REQUIRE(reader.read_pod<double>() == 3.14159265358979);
    REQUIRE(reader.at_end());
}

TEST_CASE("ByteWriter/ByteReader round-trip a POD array", "[serialization]") {
    const std::vector<Point> points{{1.0, 2.0}, {3.0, 4.0}, {5.0, 6.0}};

    ByteWriter writer;
    writer.write_pod_array<Point>(std::span<const Point>(points));

    ByteReader reader(writer.bytes());
    const auto read_back = reader.read_pod_array<Point>();
    REQUIRE(read_back == points);
    REQUIRE(reader.at_end());
}

TEST_CASE("ByteWriter/ByteReader round-trip an empty POD array", "[serialization]") {
    ByteWriter writer;
    writer.write_pod_array<Point>(std::span<const Point>());

    ByteReader reader(writer.bytes());
    const auto read_back = reader.read_pod_array<Point>();
    REQUIRE(read_back.empty());
    REQUIRE(reader.at_end());
}

TEST_CASE("ComponentPool serialize/deserialize round-trips values and entity mapping", "[serialization][ecs]") {
    ComponentPool<Point> pool;
    const Entity a{0, 0};
    const Entity b{1, 0};
    const Entity c{5, 2};  // sparse: leaves a gap in the sparse array

    pool.emplace(a, Point{1.0, 1.0});
    pool.emplace(b, Point{2.0, 2.0});
    pool.emplace(c, Point{3.0, 3.0});

    ByteWriter writer;
    pool.serialize(writer);

    ComponentPool<Point> restored;
    ByteReader reader(writer.bytes());
    restored.deserialize(reader);

    REQUIRE(restored.size() == 3);
    REQUIRE(restored.has(a));
    REQUIRE(restored.has(b));
    REQUIRE(restored.has(c));
    REQUIRE(restored.get(a) == Point{1.0, 1.0});
    REQUIRE(restored.get(b) == Point{2.0, 2.0});
    REQUIRE(restored.get(c) == Point{3.0, 3.0});

    // Iteration order preserved too.
    std::vector<Entity> order;
    for (const Entity e : restored.entities()) {
        order.push_back(e);
    }
    REQUIRE(order == std::vector<Entity>{a, b, c});
}

TEST_CASE("ComponentPool deserialize replaces prior contents", "[serialization][ecs]") {
    ComponentPool<Point> source;
    source.emplace(Entity{0, 0}, Point{9.0, 9.0});

    ByteWriter writer;
    source.serialize(writer);

    ComponentPool<Point> target;
    target.emplace(Entity{7, 0}, Point{-1.0, -1.0});  // should be wiped by deserialize

    ByteReader reader(writer.bytes());
    target.deserialize(reader);

    REQUIRE(target.size() == 1);
    REQUIRE_FALSE(target.has(Entity{7, 0}));
    REQUIRE(target.has(Entity{0, 0}));
}
