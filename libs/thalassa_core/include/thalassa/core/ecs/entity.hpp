#pragma once

#include <cstdint>

#include "thalassa/core/types.hpp"

// Entity handles pack a generation counter alongside the index so a stale
// handle to a destroyed-and-recycled entity slot is detectable rather than
// silently aliasing a new entity — important once networking/prediction
// holds onto entity references across ticks.

namespace thalassa::core::ecs {

class Entity {
public:
    using IndexType = std::uint32_t;
    using GenerationType = std::uint32_t;

    static constexpr IndexType kInvalidIndex = 0xFFFFFFFFu;

    constexpr Entity() noexcept = default;
    constexpr Entity(IndexType index, GenerationType generation) noexcept
        : index_(index), generation_(generation) {}

    [[nodiscard]] constexpr IndexType index() const noexcept { return index_; }
    [[nodiscard]] constexpr GenerationType generation() const noexcept { return generation_; }
    [[nodiscard]] constexpr bool is_valid() const noexcept { return index_ != kInvalidIndex; }

    constexpr bool operator==(const Entity&) const noexcept = default;

private:
    IndexType index_ = kInvalidIndex;
    GenerationType generation_ = 0;
};

}  // namespace thalassa::core::ecs

template <>
struct std::hash<thalassa::core::ecs::Entity> {
    std::size_t operator()(const thalassa::core::ecs::Entity& e) const noexcept {
        const std::uint64_t combined =
            (static_cast<std::uint64_t>(e.index()) << 32) | static_cast<std::uint64_t>(e.generation());
        return std::hash<std::uint64_t>{}(combined);
    }
};
