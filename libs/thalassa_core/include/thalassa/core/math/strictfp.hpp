#pragma once

#include "thalassa/core/types.hpp"

// strictfp: deterministic scalar math for anything that touches simulation
// state. See docs/floating_point_policy.md for why this exists instead of
// calling into <cmath> directly from sim code.
//
// sqrt (Newton-Raphson) and a polynomial sin/cos
// (minimax approximation, range-reduced) sufficient for steering/rotation. 
// These are intentionally simple, single
// well-defined algorithms (no libm dispatch, no runtime CPU-feature
// branching) so that the *same* result comes out on every machine we build
// for. Accuracy is validated in tests/thalassa_core/strictfp_tests.cpp
// against <cmath> to a documented tolerance; determinism is validated by
// bit-for-bit comparison across repeated calls, not against libm.

namespace thalassa::core::math::strictfp {

// Deterministic square root via Newton-Raphson with a fixed iteration count
// and a bit-trick initial guess. Fixed iteration count (not "until
// converged") is the important part: a data-dependent loop trip count is a
// latent determinism hazard if two builds ever disagree on convergence
// (e.g. different iteration due to different rounding at the last bit).
[[nodiscard]] Scalar sqrt(Scalar x) noexcept;

// Range-reduced, fixed-iteration polynomial approximations. Valid for any
// finite input; internally reduces to [-pi, pi] using a fixed constant for
// pi (not std::numbers::pi_v, to guarantee the literal bit pattern is
// identical across translation units/compilers).
[[nodiscard]] Scalar sin(Scalar radians) noexcept;
[[nodiscard]] Scalar cos(Scalar radians) noexcept;

// atan2 via a fixed-degree minimax polynomial + quadrant fixup. Used for
// facing/steering computations. Worst-case error ~1.7e-5 radians against
// std::atan2 (see strictfp.cpp for the source of the approximation and
// strictfp_tests.cpp for the empirical bound) — adequate for gameplay
// facing/steering, not for high-precision geometry.
[[nodiscard]] Scalar atan2(Scalar y, Scalar x) noexcept;

// Constants pinned as exact literals rather than derived at runtime.
inline constexpr Scalar kPi        = 3.14159265358979323846;
inline constexpr Scalar kTwoPi     = 6.28318530717958647692;
inline constexpr Scalar kHalfPi    = 1.57079632679489661923;

}  // namespace thalassa::core::math::strictfp
