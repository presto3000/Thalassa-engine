#pragma once

#include "thalassa/core/math/vector.hpp"
#include "thalassa/core/types.hpp"

// Coordinate/unit conversion between Thalassa's internal convention and
// Unreal Engine's:
//
//   Thalassa: right-handed, Z-up, meters.       X-forward, Y-left.
//   Unreal:   left-handed, Z-up, centimeters.   X-forward, Y-right.
//
// Deliberately plain functions over plain Vec3/floats — no Unreal types
// appear here (no FVector, no dependency on Unreal headers at all), so
// this file compiles and is unit-testable with the same toolchain as the
// rest of this repo, unlike the actual UE5 plugin code that calls it. See
// the design doc's "What's verified here, and what isn't" section.
//
// UE5's own FVector uses float (single precision); Thalassa's Scalar is
// double. These conversions
// deliberately take/return plain double triples (x, y, z) rather than a
// Vec3 on the "Unreal side," so the UE5-facing call site (which does have
// FVector) does the final narrowing to float explicitly and visibly,
// rather than this header silently doing a precision-losing conversion
// inside a function whose name doesn't suggest it.

namespace thalassa::bridge {

inline constexpr double kCentimetersPerMeter = 100.0;

struct UnrealVector {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// Thalassa (meters, right-handed) -> Unreal (centimeters, left-handed).
[[nodiscard]] inline UnrealVector to_unreal(const core::math::Vec3& thalassa_position) noexcept {
    return UnrealVector{
        thalassa_position.x * kCentimetersPerMeter,
        -thalassa_position.y * kCentimetersPerMeter,
        thalassa_position.z * kCentimetersPerMeter,
    };
}

// Unreal (centimeters, left-handed) -> Thalassa (meters, right-handed).
[[nodiscard]] inline core::math::Vec3 to_thalassa(const UnrealVector& unreal_position) noexcept {
    return core::math::Vec3{
        unreal_position.x / kCentimetersPerMeter,
        -unreal_position.y / kCentimetersPerMeter,
        unreal_position.z / kCentimetersPerMeter,
    };
}

// Scalar-only conversions for values that aren't positions/directions
// (speeds, radii, ranges) — same unit scale, no handedness flip since
// there's no axis to flip for a plain magnitude.
[[nodiscard]] constexpr double meters_to_unreal_units(core::Scalar meters) noexcept {
    return static_cast<double>(meters) * kCentimetersPerMeter;
}
[[nodiscard]] constexpr core::Scalar unreal_units_to_meters(double unreal_units) noexcept {
    return static_cast<core::Scalar>(unreal_units / kCentimetersPerMeter);
}

// Computes the Unreal yaw (degrees, rotation about Z from +X) that
// corresponds to facing along `thalassa_direction` in Thalassa's frame.
// Thalassa doesn't track orientation as its own component (see
// components.hpp's Position doc comment), so presentation code commonly
// wants to derive a facing direction from Velocity instead — this is that
// conversion, done once and tested here rather than reimplemented at every
// UE5-side call site.
//
// The handedness flip means this is NOT simply "the angle of the
// direction vector": in Thalassa's right-handed frame, facing angle
// theta = atan2(dy, dx); under Unreal's left-handed frame (Y flipped),
// the same physical direction has Unreal yaw = -theta. Returns 0 for a
// (near-)zero direction, since there's no meaningful facing to report —
// callers that want "keep the last known facing" instead of "snap to 0"
// need to hold onto the previous value themselves; this is a pure
// function with no memory of prior calls.
[[nodiscard]] inline double facing_yaw_degrees(const core::math::Vec3& thalassa_direction) noexcept {
    if (thalassa_direction.length_sq() <= 0.0) {
        return 0.0;
    }
    const core::Scalar thalassa_yaw_radians =
        core::math::strictfp::atan2(thalassa_direction.y, thalassa_direction.x);
    const core::Scalar unreal_yaw_radians = -thalassa_yaw_radians;
    return static_cast<double>(unreal_yaw_radians) * (180.0 / core::math::strictfp::kPi);
}

}  // namespace thalassa::bridge
