#pragma once

#include <cstdint>

#include "thalassa/core/assert.hpp"
#include "thalassa/core/types.hpp"

// Deterministic RNG for simulation use. PCG32 (O'Neill, 2014): small state,
// fast, good statistical quality, and — the property that actually matters
// here — a fully specified integer algorithm with no platform-dependent
// behavior, unlike std::mt19937's implementation-defined-adjacent seeding
// or any RNG that touches floating point internally. Every world/replay
// carries its own Rng instance seeded explicitly (never from
// std::random_device or time) so replays are reproducible byte-for-byte.

namespace thalassa::core {

class Rng {
public:
    constexpr Rng() noexcept : Rng(0x853c49e6748fea9bULL) {}

    constexpr explicit Rng(std::uint64_t seed, std::uint64_t stream = 1) noexcept {
        state_ = 0;
        inc_ = (stream << 1u) | 1u;
        next_u32();
        state_ += seed;
        next_u32();
    }

    // Returns a uniformly distributed 32-bit value covering the full range.
    constexpr std::uint32_t next_u32() noexcept {
        const std::uint64_t old_state = state_;
        state_ = old_state * 6364136223846793005ULL + inc_;
        const auto xorshifted = static_cast<std::uint32_t>(((old_state >> 18u) ^ old_state) >> 27u);
        const auto rot = static_cast<std::uint32_t>(old_state >> 59u);
        return (xorshifted >> rot) | (xorshifted << ((~rot + 1u) & 31u));
    }

    // Returns a uniformly distributed 64-bit value by combining two draws.
    constexpr std::uint64_t next_u64() noexcept {
        const std::uint64_t hi = next_u32();
        const std::uint64_t lo = next_u32();
        return (hi << 32) | lo;
    }

    // [0, 1) uniform Scalar. Uses the top 53 bits of a 64-bit draw so the
    // full double mantissa is populated (avoids the classic
    // rand()/RAND_MAX low-bit-quality trap).
    [[nodiscard]] Scalar next_scalar01() noexcept {
        constexpr std::uint64_t kMantissaBits = 53;
        const std::uint64_t bits = next_u64() >> (64 - kMantissaBits);
        return static_cast<Scalar>(bits) / static_cast<Scalar>(std::uint64_t{1} << kMantissaBits);
    }

    // Uniform integer in [min, max] inclusive, using Lemire's method to
    // avoid modulo bias.
    [[nodiscard]] std::uint32_t next_range(std::uint32_t min, std::uint32_t max) noexcept {
        THALASSA_ASSERT(min <= max);
        const std::uint32_t range = max - min + 1u;
        if (range == 0) {
            // max - min + 1 wrapped to 0 only when [min,max] spans the
            // full uint32 domain; treat as "any value" in that case.
            return next_u32();
        }
        std::uint64_t multiresult = static_cast<std::uint64_t>(next_u32()) * static_cast<std::uint64_t>(range);
        auto leftover = static_cast<std::uint32_t>(multiresult);
        if (leftover < range) {
            const std::uint32_t threshold = (~range + 1u) % range;
            while (leftover < threshold) {
                multiresult = static_cast<std::uint64_t>(next_u32()) * static_cast<std::uint64_t>(range);
                leftover = static_cast<std::uint32_t>(multiresult);
            }
        }
        return min + static_cast<std::uint32_t>(multiresult >> 32);
    }

    // Snapshots/restores state for serialization (replay, save games, and
    // client<->server resync). Trivially copyable by design.
    [[nodiscard]] constexpr std::uint64_t state() const noexcept { return state_; }
    [[nodiscard]] constexpr std::uint64_t inc() const noexcept { return inc_; }
    constexpr void restore(std::uint64_t state, std::uint64_t inc) noexcept {
        state_ = state;
        inc_ = inc;
    }

private:
    std::uint64_t state_;
    std::uint64_t inc_;
};

}  // namespace thalassa::core
