#pragma once

#include "thalassa/core/math/strictfp.hpp"
#include "thalassa/core/types.hpp"

// Deterministic vector/quaternion math for simulation state. All operations
// here are pure functions over Scalar (see types.hpp) using strictfp for
// anything transcendental, so the same inputs always produce the same
// outputs bit-for-bit given the compiler flags in
// cmake/DeterministicMath.cmake. This header has zero platform or rendering
// dependencies and is safe to include from Unreal Engine module code (it
// converts to/from FVector at the boundary in thalassa_render or the UE5
// integration layer, never here).

namespace thalassa::core::math {

struct Vec2 {
    Scalar x = 0.0;
    Scalar y = 0.0;

    constexpr Vec2() noexcept = default;
    constexpr Vec2(Scalar x_, Scalar y_) noexcept : x(x_), y(y_) {}

    [[nodiscard]] constexpr Vec2 operator+(const Vec2& rhs) const noexcept { return {x + rhs.x, y + rhs.y}; }
    [[nodiscard]] constexpr Vec2 operator-(const Vec2& rhs) const noexcept { return {x - rhs.x, y - rhs.y}; }
    [[nodiscard]] constexpr Vec2 operator*(Scalar s) const noexcept { return {x * s, y * s}; }
    [[nodiscard]] constexpr Vec2 operator-() const noexcept { return {-x, -y}; }

    constexpr Vec2& operator+=(const Vec2& rhs) noexcept { x += rhs.x; y += rhs.y; return *this; }
    constexpr Vec2& operator-=(const Vec2& rhs) noexcept { x -= rhs.x; y -= rhs.y; return *this; }
    constexpr Vec2& operator*=(Scalar s) noexcept { x *= s; y *= s; return *this; }

    constexpr bool operator==(const Vec2&) const noexcept = default;

    [[nodiscard]] constexpr Scalar dot(const Vec2& rhs) const noexcept { return x * rhs.x + y * rhs.y; }
    [[nodiscard]] constexpr Scalar length_sq() const noexcept { return dot(*this); }
    [[nodiscard]] Scalar length() const noexcept { return strictfp::sqrt(length_sq()); }

    [[nodiscard]] Vec2 normalized() const noexcept {
        const Scalar len = length();
        if (len <= 0.0) {
            return {0.0, 0.0};
        }
        return {x / len, y / len};
    }
};

[[nodiscard]] constexpr Vec2 operator*(Scalar s, const Vec2& v) noexcept { return v * s; }

struct Vec3 {
    Scalar x = 0.0;
    Scalar y = 0.0;
    Scalar z = 0.0;

    constexpr Vec3() noexcept = default;
    constexpr Vec3(Scalar x_, Scalar y_, Scalar z_) noexcept : x(x_), y(y_), z(z_) {}

    [[nodiscard]] constexpr Vec3 operator+(const Vec3& rhs) const noexcept { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
    [[nodiscard]] constexpr Vec3 operator-(const Vec3& rhs) const noexcept { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
    [[nodiscard]] constexpr Vec3 operator*(Scalar s) const noexcept { return {x * s, y * s, z * s}; }
    [[nodiscard]] constexpr Vec3 operator-() const noexcept { return {-x, -y, -z}; }

    constexpr Vec3& operator+=(const Vec3& rhs) noexcept { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
    constexpr Vec3& operator-=(const Vec3& rhs) noexcept { x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; }
    constexpr Vec3& operator*=(Scalar s) noexcept { x *= s; y *= s; z *= s; return *this; }

    constexpr bool operator==(const Vec3&) const noexcept = default;

    [[nodiscard]] constexpr Scalar dot(const Vec3& rhs) const noexcept { return x * rhs.x + y * rhs.y + z * rhs.z; }
    [[nodiscard]] constexpr Vec3 cross(const Vec3& rhs) const noexcept {
        return {
            y * rhs.z - z * rhs.y,
            z * rhs.x - x * rhs.z,
            x * rhs.y - y * rhs.x,
        };
    }
    [[nodiscard]] constexpr Scalar length_sq() const noexcept { return dot(*this); }
    [[nodiscard]] Scalar length() const noexcept { return strictfp::sqrt(length_sq()); }

    [[nodiscard]] Vec3 normalized() const noexcept {
        const Scalar len = length();
        if (len <= 0.0) {
            return {0.0, 0.0, 0.0};
        }
        return {x / len, y / len, z / len};
    }
};

[[nodiscard]] constexpr Vec3 operator*(Scalar s, const Vec3& v) noexcept { return v * s; }

// Quaternion (w, x, y, z) for deterministic orientation. Multiplication and
// normalization for now; more (slerp,
// axis-angle) will be added alongside the movement/steering systems in.
struct Quat {
    Scalar w = 1.0;
    Scalar x = 0.0;
    Scalar y = 0.0;
    Scalar z = 0.0;

    constexpr Quat() noexcept = default;
    constexpr Quat(Scalar w_, Scalar x_, Scalar y_, Scalar z_) noexcept : w(w_), x(x_), y(y_), z(z_) {}

    [[nodiscard]] constexpr Quat operator*(const Quat& rhs) const noexcept {
        return {
            w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
            w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
            w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
            w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w,
        };
    }

    [[nodiscard]] constexpr Scalar length_sq() const noexcept { return w * w + x * x + y * y + z * z; }
    [[nodiscard]] Scalar length() const noexcept { return strictfp::sqrt(length_sq()); }

    [[nodiscard]] Quat normalized() const noexcept {
        const Scalar len = length();
        if (len <= 0.0) {
            return {1.0, 0.0, 0.0, 0.0};
        }
        return {w / len, x / len, y / len, z / len};
    }

    [[nodiscard]] static Quat from_axis_angle(const Vec3& axis, Scalar radians) noexcept {
        const Vec3 n = axis.normalized();
        const Scalar half = radians * 0.5;
        const Scalar s = strictfp::sin(half);
        return Quat{strictfp::cos(half), n.x * s, n.y * s, n.z * s};
    }
};

}  // namespace thalassa::core::math
