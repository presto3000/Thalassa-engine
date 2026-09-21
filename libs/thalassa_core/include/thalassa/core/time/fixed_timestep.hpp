#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <thread>

#include "thalassa/core/types.hpp"

// Fixed timestep loop: the ONLY way simulation code is ever ticked in this
// project (coding standard: "Fixed timestep only. Variable delta time is
// forbidden in simulation."). Wall-clock time is used purely to decide *how
// many* fixed ticks to run and whether to sleep — it never leaks into the
// simulation as a delta; every simulation tick advances by exactly
// `tick_duration()`.

namespace thalassa::core::time {

using Clock = std::chrono::steady_clock;

// Fixed simulation rate. 60 Hz by default per architecture spec ("30 or 60
// Hz"); constructible with a different rate for tests/tools.
class FixedTimestep {
public:
    explicit FixedTimestep(std::uint32_t ticks_per_second = 60) noexcept
        : ticks_per_second_(ticks_per_second),
          tick_duration_(std::chrono::duration_cast<Clock::duration>(
              std::chrono::duration<double>(1.0 / static_cast<double>(ticks_per_second)))) {}

    [[nodiscard]] std::uint32_t ticks_per_second() const noexcept { return ticks_per_second_; }
    [[nodiscard]] Clock::duration tick_duration() const noexcept { return tick_duration_; }
    [[nodiscard]] Scalar tick_duration_seconds() const noexcept {
        return 1.0 / static_cast<Scalar>(ticks_per_second_);
    }

    // Caps how many ticks a single call to advance() will run even if wall
    // clock time jumped further (debugger pause, OS suspend/resume, huge
    // frame hitch). Prevents a "spiral of death" where catching up takes
    // longer than real time, at the cost of the simulation falling behind
    // wall-clock in that pathological case — an explicit, logged tradeoff
    // rather than an unbounded catch-up loop.
    static constexpr std::uint32_t kMaxTicksPerAdvance = 8;

    // Called once per outer loop iteration. Invokes `on_tick(SimTick)`
    // zero or more times (accumulator pattern), always in tick order,
    // always with exactly tick_duration() of simulated time per call.
    // Returns the number of ticks actually run this call (useful for
    // callers that want to know if they fell behind).
    template <typename TickFn>
    std::uint32_t advance(TickFn&& on_tick) {
        const Clock::time_point now = Clock::now();
        accumulator_ += (now - last_time_);
        last_time_ = now;

        std::uint32_t ticks_run = 0;
        while (accumulator_ >= tick_duration_ && ticks_run < kMaxTicksPerAdvance) {
            on_tick(current_tick_);
            current_tick_ = current_tick_ + 1;
            accumulator_ -= tick_duration_;
            ++ticks_run;
        }

        // If we hit the cap while still behind, drop the remaining
        // accumulated debt rather than let it compound across frames.
        if (ticks_run == kMaxTicksPerAdvance && accumulator_ >= tick_duration_) {
            accumulator_ = Clock::duration::zero();
        }
        return ticks_run;
    }

    // Fraction in [0, 1) of the way into the next, not-yet-simulated tick.
    // For presentation-layer interpolation only — never fed back into
    // simulation state.
    [[nodiscard]] double interpolation_alpha() const noexcept {
        return std::chrono::duration<double>(accumulator_) / std::chrono::duration<double>(tick_duration_);
    }

    [[nodiscard]] SimTick current_tick() const noexcept { return current_tick_; }

    void reset(Clock::time_point now = Clock::now()) noexcept {
        last_time_ = now;
        accumulator_ = Clock::duration::zero();
        current_tick_ = SimTick{0};
    }

private:
    std::uint32_t ticks_per_second_;
    Clock::duration tick_duration_;
    Clock::time_point last_time_{Clock::now()};
    Clock::duration accumulator_{Clock::duration::zero()};
    SimTick current_tick_{0};
};

// Sleeps the calling thread until roughly `target` (headless server /
// dedicated process use — avoids busy-spinning a CPU core at 60Hz).
// Deliberately imprecise (relies on OS scheduler); fine for our purposes
// since simulation correctness never depends on wall-clock precision, only
// on tick *count* and *order*.
inline void sleep_until(Clock::time_point target) {
    std::this_thread::sleep_until(target);
}

}  // namespace thalassa::core::time
