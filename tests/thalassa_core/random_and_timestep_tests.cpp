#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <thread>

#include "thalassa/core/random.hpp"
#include "thalassa/core/time/fixed_timestep.hpp"

using thalassa::core::Rng;
using thalassa::core::time::FixedTimestep;

TEST_CASE("Rng with the same seed produces the same sequence", "[random]") {
    Rng a(12345);
    Rng b(12345);
    for (int i = 0; i < 100; ++i) {
        REQUIRE(a.next_u32() == b.next_u32());
    }
}

TEST_CASE("Rng with different seeds diverges", "[random]") {
    Rng a(1);
    Rng b(2);
    bool any_different = false;
    for (int i = 0; i < 10; ++i) {
        if (a.next_u32() != b.next_u32()) {
            any_different = true;
        }
    }
    REQUIRE(any_different);
}

TEST_CASE("Rng next_scalar01 stays in [0, 1)", "[random]") {
    Rng rng(7);
    for (int i = 0; i < 10000; ++i) {
        const double v = rng.next_scalar01();
        REQUIRE(v >= 0.0);
        REQUIRE(v < 1.0);
    }
}

TEST_CASE("Rng next_range stays within [min, max]", "[random]") {
    Rng rng(99);
    for (int i = 0; i < 10000; ++i) {
        const std::uint32_t v = rng.next_range(5, 10);
        REQUIRE(v >= 5);
        REQUIRE(v <= 10);
    }
}

TEST_CASE("Rng state can be snapshotted and restored", "[random]") {
    Rng rng(555);
    for (int i = 0; i < 50; ++i) {
        (void)rng.next_u32();
    }
    const std::uint64_t state = rng.state();
    const std::uint64_t inc = rng.inc();

    const std::uint32_t next_before = rng.next_u32();

    Rng restored;
    restored.restore(state, inc);
    const std::uint32_t next_after = restored.next_u32();

    REQUIRE(next_before == next_after);
}

TEST_CASE("FixedTimestep never runs more than kMaxTicksPerAdvance ticks in one advance() call", "[timestep]") {
    FixedTimestep ts(60);
    ts.reset(thalassa::core::time::Clock::now() - std::chrono::seconds(10));  // simulate a huge stall

    int total_ticks = 0;
    const std::uint32_t ran = ts.advance([&total_ticks](thalassa::core::SimTick) { ++total_ticks; });

    REQUIRE(ran <= FixedTimestep::kMaxTicksPerAdvance);
    REQUIRE(total_ticks == static_cast<int>(ran));
}

TEST_CASE("FixedTimestep advances tick counter monotonically", "[timestep]") {
    FixedTimestep ts(1000);  // fast rate so the test doesn't need to sleep long
    ts.reset();

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    const std::uint32_t ran = ts.advance([](thalassa::core::SimTick) {});
    REQUIRE(ran >= 1);
    REQUIRE(thalassa::core::to_u64(ts.current_tick()) == ran);
}
