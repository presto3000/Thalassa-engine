#pragma once

#include <cstdint>

#include "thalassa/core/ecs/entity.hpp"

// Packs/unpacks core::ecs::Entity (a 32-bit index + 32-bit generation) to
// and from a form suitable for a UE5 USTRUCT(BlueprintType) ("FThalassaEntityHandle").
// Kept here (not in the UE5 plugin itself) so the pack/unpack round-trip
// is unit-tested against the real Entity type, rather than only ever
// exercised inside UE5-specific code this environment can't compile.
//
// Represented as two plain std::int32_t (not std::uint32_t) specifically
// because that's the field type UE5's reflection system supports natively
// as an int32 UPROPERTY — Entity's own IndexType/GenerationType are
// unsigned, so this is a reinterpretation, not a value-range-losing
// narrowing (every valid std::uint32_t bit pattern round-trips exactly
// through std::int32_t and back).

namespace thalassa::bridge {

struct PackedEntityHandle {
    std::int32_t index = -1;
    std::int32_t generation = 0;
};

[[nodiscard]] inline PackedEntityHandle pack_entity(core::ecs::Entity entity) noexcept {
    return PackedEntityHandle{
        static_cast<std::int32_t>(entity.index()),
        static_cast<std::int32_t>(entity.generation()),
    };
}

[[nodiscard]] inline core::ecs::Entity unpack_entity(PackedEntityHandle handle) noexcept {
    return core::ecs::Entity{
        static_cast<core::ecs::Entity::IndexType>(handle.index),
        static_cast<core::ecs::Entity::GenerationType>(handle.generation),
    };
}

}  // namespace thalassa::bridge
