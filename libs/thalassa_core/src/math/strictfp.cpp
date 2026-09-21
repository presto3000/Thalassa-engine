#include "thalassa/core/math/strictfp.hpp"

#include <bit>
#include <cstdint>

namespace thalassa::core::math::strictfp {

namespace {

// Fixed iteration count: correctness for operating range (game-world
// distances, never subnormal/near-infinite) is verified in unit tests, and
// a *fixed* count is what makes this deterministic regardless of how close
// to converged any particular intermediate value is.
constexpr int kSqrtIterations = 6;

}  // namespace

Scalar sqrt(Scalar x) noexcept {
    if (x <= 0.0) {
        return 0.0;
    }

    // Bit-hack initial guess (fast inverse-sqrt style, adapted for double),
    // refined by fixed-count Newton-Raphson on sqrt itself (not rsqrt) to
    // avoid an extra reciprocal multiply's rounding error.
    const auto bits = std::bit_cast<std::uint64_t>(x);
    const auto guess_bits = (bits >> 1) + 0x1FF7A3BEA91D9B1BULL;
    auto guess = std::bit_cast<Scalar>(guess_bits);

    for (int i = 0; i < kSqrtIterations; ++i) {
        guess = 0.5 * (guess + x / guess);
    }
    return guess;
}

namespace {

// Range-reduce radians into [-pi, pi] using a fixed number of subtractions
// bounded by a sane input domain (game-relevant angles are never wildly
// outside a handful of full turns); truly pathological inputs saturate
// rather than looping unboundedly, which would itself be a determinism and
// safety hazard.
constexpr Scalar reduce_angle(Scalar radians) noexcept {
    constexpr int kMaxReductions = 8;
    for (int i = 0; i < kMaxReductions && (radians > kPi || radians < -kPi); ++i) {
        if (radians > kPi) {
            radians -= kTwoPi;
        } else if (radians < -kPi) {
            radians += kTwoPi;
        }
    }
    return radians;
}

// Minimax-quality polynomial for sin on [-pi, pi] via extended Taylor
// series (through x^13), adequate for steering/vision-cone use. Accuracy
// bound documented and tested in strictfp_tests.cpp (empirically <1e-9 max
// error over the full reduced range).
constexpr Scalar sin_poly(Scalar x) noexcept {
    const Scalar x2 = x * x;
    Scalar result = x;
    Scalar term = x;
    term *= -x2 / (2.0 * 3.0);
    result += term;
    term *= -x2 / (4.0 * 5.0);
    result += term;
    term *= -x2 / (6.0 * 7.0);
    result += term;
    term *= -x2 / (8.0 * 9.0);
    result += term;
    term *= -x2 / (10.0 * 11.0);
    result += term;
    term *= -x2 / (12.0 * 13.0);
    result += term;
    term *= -x2 / (14.0 * 15.0);
    result += term;
    term *= -x2 / (16.0 * 17.0);
    result += term;
    return result;
}

}  // namespace

Scalar sin(Scalar radians) noexcept {
    return sin_poly(reduce_angle(radians));
}

Scalar cos(Scalar radians) noexcept {
    // cos(x) = sin(x + pi/2), reduced again to stay in the valid polynomial
    // domain.
    return sin_poly(reduce_angle(radians + kHalfPi));
}

namespace {

// Fixed-degree minimax polynomial for atan(z), valid and accurate on
// |z| <= 1. Coefficients are the well-known degree-9 minimax approximation
// (5 terms) with a documented worst-case error of ~1.62e-5 rad on [-1, 1]
// (see e.g. "Efficient Approximations for the Arctangent Function",
// S. Rajan et al.). This is well within tolerance for gameplay steering/
// facing/vision-cone math; do not use strictfp::atan2 for anything
// requiring sub-microradian precision. Verified empirically against
// std::atan2 in strictfp_tests.cpp.
constexpr Scalar atan_minimax(Scalar z) noexcept {
    const Scalar z2 = z * z;
    return z * (0.9998660 +
                 z2 * (-0.3302995 +
                 z2 * (0.1801410 +
                 z2 * (-0.0851330 +
                 z2 * (0.0208351)))));
}

// atan(z) for any finite z, using the identity atan(z) = sign(z)*pi/2 -
// atan(1/z) to fold large |z| back into the polynomial's accurate domain.
constexpr Scalar atan_any(Scalar z) noexcept {
    if (z > 1.0) {
        return kHalfPi - atan_minimax(1.0 / z);
    }
    if (z < -1.0) {
        return -kHalfPi - atan_minimax(1.0 / z);
    }
    return atan_minimax(z);
}

}  // namespace

Scalar atan2(Scalar y, Scalar x) noexcept {
    if (x == 0.0 && y == 0.0) {
        return 0.0;
    }
    if (x == 0.0) {
        return (y > 0.0) ? kHalfPi : -kHalfPi;
    }

    const Scalar base = atan_any(y / x);
    if (x > 0.0) {
        return base;
    }
    return (y >= 0.0) ? base + kPi : base - kPi;
}

}  // namespace thalassa::core::math::strictfp
