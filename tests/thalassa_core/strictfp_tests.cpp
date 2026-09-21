#include <cmath>
#include <utility>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "thalassa/core/math/strictfp.hpp"

using Catch::Approx;
using namespace thalassa::core::math;

TEST_CASE("strictfp::sqrt matches std::sqrt within tolerance", "[strictfp]") {
    const double inputs[] = {0.0, 1.0, 2.0, 4.0, 100.0, 0.0001, 123456.789};
    for (const double x : inputs) {
        REQUIRE(strictfp::sqrt(x) == Approx(std::sqrt(x)).epsilon(1e-10));
    }
}

TEST_CASE("strictfp::sqrt is deterministic across repeated calls", "[strictfp]") {
    for (int i = 0; i < 1000; ++i) {
        REQUIRE(strictfp::sqrt(2.0) == strictfp::sqrt(2.0));
    }
}

TEST_CASE("strictfp::sin/cos match std within tolerance across a range", "[strictfp]") {
    for (int deg = -720; deg <= 720; deg += 15) {
        const double radians = static_cast<double>(deg) * strictfp::kPi / 180.0;
        REQUIRE(strictfp::sin(radians) == Approx(std::sin(radians)).margin(1e-6));
        REQUIRE(strictfp::cos(radians) == Approx(std::cos(radians)).margin(1e-6));
    }
}

TEST_CASE("strictfp::atan2 matches std::atan2 across all quadrants", "[strictfp]") {
    const std::pair<double, double> cases[] = {
        {1.0, 1.0}, {1.0, -1.0}, {-1.0, 1.0}, {-1.0, -1.0},
        {0.0, 1.0}, {0.0, -1.0}, {1.0, 0.0}, {-1.0, 0.0},
        {5.0, 0.001}, {0.001, 5.0}, {-5.0, 0.001}, {0.001, -5.0},
    };
    // Margin reflects the documented worst-case error of the degree-9
    // minimax approximation used internally (~1.7e-5 rad) — see
    // strictfp.cpp. Not a precision guarantee for non-gameplay geometry.
    for (const auto& [y, x] : cases) {
        REQUIRE(strictfp::atan2(y, x) == Approx(std::atan2(y, x)).margin(2e-5));
    }
}

TEST_CASE("strictfp::atan2(0,0) returns 0 without NaN", "[strictfp]") {
    REQUIRE(strictfp::atan2(0.0, 0.0) == 0.0);
}
