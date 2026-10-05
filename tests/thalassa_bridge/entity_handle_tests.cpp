#include <cstdint>
#include <limits>

#include <catch2/catch_test_macros.hpp>

#include "thalassa/bridge/entity_handle.hpp"

using thalassa::bridge::pack_entity;
using thalassa::bridge::PackedEntityHandle;
using thalassa::bridge::unpack_entity;
using thalassa::core::ecs::Entity;

TEST_CASE("pack_entity/unpack_entity round-trip an ordinary entity", "[entity_handle]") {
    const Entity original{42, 7};
    const PackedEntityHandle packed = pack_entity(original);
    const Entity round_tripped = unpack_entity(packed);

    REQUIRE(round_tripped == original);
    REQUIRE(round_tripped.index() == 42);
    REQUIRE(round_tripped.generation() == 7);
}

TEST_CASE("pack_entity/unpack_entity round-trip the default (invalid) entity", "[entity_handle]") {
    const Entity original{};  // default-constructed, is_valid() == false
    const Entity round_tripped = unpack_entity(pack_entity(original));

    REQUIRE_FALSE(round_tripped.is_valid());
    REQUIRE(round_tripped == original);
}

TEST_CASE("pack_entity/unpack_entity round-trip large index/generation values without truncation",
          "[entity_handle]") {
    // Entity::IndexType/GenerationType are uint32_t; values whose top bit
    // is set are exactly where a naive signed/unsigned mismatch would
    // truncate or reinterpret incorrectly.
    constexpr std::uint32_t kLargeIndex = 0x80000001u;
    constexpr std::uint32_t kLargeGeneration = 0xFFFFFFFEu;

    const Entity original{kLargeIndex, kLargeGeneration};
    const Entity round_tripped = unpack_entity(pack_entity(original));

    REQUIRE(round_tripped.index() == kLargeIndex);
    REQUIRE(round_tripped.generation() == kLargeGeneration);
}

TEST_CASE("Distinct entities pack to distinct handles", "[entity_handle]") {
    const Entity a{1, 0};
    const Entity b{1, 1};  // same index, different generation
    const Entity c{2, 0};  // different index, same generation as a

    const PackedEntityHandle packed_a = pack_entity(a);
    const PackedEntityHandle packed_b = pack_entity(b);
    const PackedEntityHandle packed_c = pack_entity(c);

    REQUIRE_FALSE((packed_a.index == packed_b.index && packed_a.generation == packed_b.generation));
    REQUIRE_FALSE((packed_a.index == packed_c.index && packed_a.generation == packed_c.generation));
}
