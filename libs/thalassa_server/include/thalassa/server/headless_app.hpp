#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>

#include "thalassa/core/time/fixed_timestep.hpp"
#include "thalassa/sim/sim_world.hpp"

// HeadlessApp: "Headless application that runs
// a fixed timestep loop with an empty ECS world." Runs entirely without
// rendering or windowing, suitable both as the eventual dedicated server
// entry point (later adds networking on top of this) and as a
// starting point for automated determinism tests (run twice, compare
// resulting world state).

namespace thalassa::server {

struct HeadlessAppConfig {
    std::uint32_t ticks_per_second = 60;
    std::uint64_t rng_seed = 1;

    // 0 = run until stop() is called (real dedicated server behavior).
    // Non-zero = run exactly this many sim ticks then return (used by
    // tests and by the sandbox app for quick smoke runs).
    std::uint64_t max_ticks = 0;

    // When true, sleeps between iterations to track wall-clock time
    // (real server behavior). When false, runs ticks back-to-back as fast
    // as possible with no sleeping (used for fast-forward determinism
    // tests/replays where wall-clock pacing doesn't matter).
    bool real_time = true;
};

class HeadlessApp {
public:
    explicit HeadlessApp(HeadlessAppConfig config)
        : config_(config),
          timestep_(config.ticks_per_second),
          sim_world_(config.rng_seed) {}

    // Runs the fixed-timestep loop until stop() is called from another
    // thread/signal handler, or until config_.max_ticks is reached (if
    // nonzero). Blocking call — intended to be the entire body of main()
    // for the dedicated server app.
    void run() {
        timestep_.reset();
        running_.store(true, std::memory_order_relaxed);

        while (running_.load(std::memory_order_relaxed)) {
            const std::uint32_t ticks_run = timestep_.advance([this](core::SimTick tick) {
                sim_world_.tick(tick);
            });

            if (config_.max_ticks != 0 && sim_world_.ticks_simulated() >= config_.max_ticks) {
                break;
            }

            if (config_.real_time) {
                if (ticks_run == 0) {
                    // Nothing to simulate yet this iteration; yield briefly
                    // rather than busy-spinning a CPU core at 100%.
                    core::time::sleep_until(core::time::Clock::now() + std::chrono::milliseconds(1));
                }
            }
            // real_time == false: loop again immediately (fast-forward mode).
        }
    }

    // Thread-safe; call from a signal handler or another thread to stop a
    // running() loop.
    void stop() noexcept { running_.store(false, std::memory_order_relaxed); }

    [[nodiscard]] bool running() const noexcept { return running_.load(std::memory_order_relaxed); }
    [[nodiscard]] const sim::SimWorld& sim_world() const noexcept { return sim_world_; }
    [[nodiscard]] sim::SimWorld& sim_world() noexcept { return sim_world_; }

private:
    HeadlessAppConfig config_;
    core::time::FixedTimestep timestep_;
    sim::SimWorld sim_world_;
    std::atomic<bool> running_{false};
};

}  // namespace thalassa::server
