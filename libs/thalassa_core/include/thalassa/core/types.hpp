#pragma once

#include <cstdint>
#include <functional>

#include "thalassa/core/config.hpp"

namespace thalassa::core {

// ---------------------------------------------------------------------------
// Scalar: the single type alias every simulation-affecting value must be
// expressed in. See docs/floating_point_policy.md. Swapping
// THALASSA_USE_FIXED_POINT later only requires changing this alias plus the
// contents of math/strictfp.hpp — call sites are unaffected.
// ---------------------------------------------------------------------------
#if defined(THALASSA_USE_FIXED_POINT)
    // Placeholder until a fixed-point type is implemented (see docs).
    // Intentionally not implemented in Milestone 0: policy default is IEEE-754.
    #error "THALASSA_USE_FIXED_POINT is not yet implemented; see docs/floating_point_policy.md"
#else
using Scalar = double;
#endif

// Presentation-layer values (client-only interpolation, camera, audio) are
// explicitly allowed to use plain float/double per policy; no alias needed
// there since they never cross into simulation state.

// ---------------------------------------------------------------------------
// Strong integer ID type. Prevents accidentally passing an EntityId where a
// PlayerId (etc.) is expected, and keeps every ID a trivially-copyable
// 32-bit value so components stay small and cache-friendly.
// ---------------------------------------------------------------------------
template <typename Tag>
class StrongId {
public:
    using ValueType = std::uint32_t;
    static constexpr ValueType kInvalid = 0xFFFFFFFFu;

    constexpr StrongId() noexcept : value_(kInvalid) {}
    constexpr explicit StrongId(ValueType value) noexcept : value_(value) {}

    [[nodiscard]] constexpr ValueType value() const noexcept { return value_; }
    [[nodiscard]] constexpr bool is_valid() const noexcept { return value_ != kInvalid; }

    constexpr bool operator==(const StrongId&) const noexcept = default;
    constexpr auto operator<=>(const StrongId&) const noexcept = default;

private:
    ValueType value_;
};

struct EntityTag {};
struct PlayerTag {};
struct TeamTag {};

using EntityId = StrongId<EntityTag>;
using PlayerId = StrongId<PlayerTag>;
using TeamId   = StrongId<TeamTag>;

// Simulation tick counter. 64-bit: at 60Hz this doesn't wrap for ~9.7 billion
// years, so we don't need to think about rollover.
enum class SimTick : std::uint64_t {};

[[nodiscard]] constexpr SimTick operator+(SimTick tick, std::uint64_t delta) noexcept {
    return static_cast<SimTick>(static_cast<std::uint64_t>(tick) + delta);
}

[[nodiscard]] constexpr std::uint64_t to_u64(SimTick tick) noexcept {
    return static_cast<std::uint64_t>(tick);
}

}  // namespace thalassa::core

// std::hash specializations so StrongId-based types work as unordered_map keys
// (used sparingly outside the hot sim path — see determinism rule 3 in
// docs/floating_point_policy.md; never iterate an unordered_map of these in
// simulation code without first sorting by key).
template <typename Tag>
struct std::hash<thalassa::core::StrongId<Tag>> {
    std::size_t operator()(const thalassa::core::StrongId<Tag>& id) const noexcept {
        return std::hash<typename thalassa::core::StrongId<Tag>::ValueType>{}(id.value());
    }
};
