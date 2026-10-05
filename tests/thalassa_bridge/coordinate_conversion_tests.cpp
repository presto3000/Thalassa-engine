#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "thalassa/bridge/coordinate_conversion.hpp"

using Catch::Approx;
using thalassa::bridge::to_thalassa;
using thalassa::bridge::to_unreal;
using thalassa::bridge::UnrealVector;
using thalassa::core::math::Vec3;

TEST_CASE("to_unreal converts meters to centimeters", "[coordinate_conversion]") {
    const Vec3 thalassa_pos{1.0, 0.0, 0.0};
    const UnrealVector unreal_pos = to_unreal(thalassa_pos);
    REQUIRE(unreal_pos.x == Approx(100.0));
}

TEST_CASE("to_unreal flips the Y axis (right-handed to left-handed)", "[coordinate_conversion]") {
    const Vec3 thalassa_pos{0.0, 5.0, 0.0};
    const UnrealVector unreal_pos = to_unreal(thalassa_pos);
    REQUIRE(unreal_pos.y == Approx(-500.0));
}

TEST_CASE("to_unreal leaves Z unflipped (both conventions are Z-up)", "[coordinate_conversion]") {
    const Vec3 thalassa_pos{0.0, 0.0, 3.0};
    const UnrealVector unreal_pos = to_unreal(thalassa_pos);
    REQUIRE(unreal_pos.z == Approx(300.0));
}

TEST_CASE("to_thalassa is the exact inverse of to_unreal", "[coordinate_conversion]") {
    const Vec3 original{12.5, -7.25, 3.0};
    const Vec3 round_tripped = to_thalassa(to_unreal(original));

    REQUIRE(round_tripped.x == Approx(original.x).epsilon(1e-12));
    REQUIRE(round_tripped.y == Approx(original.y).epsilon(1e-12));
    REQUIRE(round_tripped.z == Approx(original.z).epsilon(1e-12));
}

TEST_CASE("to_unreal is the exact inverse of to_thalassa", "[coordinate_conversion]") {
    const UnrealVector original{500.0, -250.0, 100.0};
    const UnrealVector round_tripped = to_unreal(to_thalassa(original));

    REQUIRE(round_tripped.x == Approx(original.x).epsilon(1e-9));
    REQUIRE(round_tripped.y == Approx(original.y).epsilon(1e-9));
    REQUIRE(round_tripped.z == Approx(original.z).epsilon(1e-9));
}

TEST_CASE("Origin maps to origin", "[coordinate_conversion]") {
    const UnrealVector unreal_origin = to_unreal(Vec3{0.0, 0.0, 0.0});
    REQUIRE(unreal_origin.x == 0.0);
    REQUIRE(unreal_origin.y == 0.0);
    REQUIRE(unreal_origin.z == 0.0);
}

TEST_CASE("meters_to_unreal_units and unreal_units_to_meters round-trip", "[coordinate_conversion]") {
    constexpr thalassa::core::Scalar original = 15.0;  // e.g. an attack range in meters
    const double as_unreal = thalassa::bridge::meters_to_unreal_units(original);
    REQUIRE(as_unreal == Approx(1500.0));

    const thalassa::core::Scalar round_tripped = thalassa::bridge::unreal_units_to_meters(as_unreal);
    REQUIRE(round_tripped == Approx(original).epsilon(1e-12));
}

TEST_CASE("A known 3D example converts exactly", "[coordinate_conversion]") {
    // A unit standing 10m forward, 4m to its (Thalassa) left, 2m up.
    const Vec3 thalassa_pos{10.0, 4.0, 2.0};
    const UnrealVector unreal_pos = to_unreal(thalassa_pos);

    REQUIRE(unreal_pos.x == Approx(1000.0));   // forward, unchanged sign, x100 scale
    REQUIRE(unreal_pos.y == Approx(-400.0));   // left becomes negative-Y under the handedness flip
    REQUIRE(unreal_pos.z == Approx(200.0));    // up, unchanged sign, x100 scale
}

TEST_CASE("facing_yaw_degrees returns 0 for facing +X (forward) in both frames", "[coordinate_conversion]") {
    REQUIRE(thalassa::bridge::facing_yaw_degrees(Vec3{1.0, 0.0, 0.0}) == Approx(0.0).margin(1e-6));
}

TEST_CASE("facing_yaw_degrees accounts for the handedness flip on +Y (Thalassa left)", "[coordinate_conversion]") {
    // Facing Thalassa's +Y (left, right-handed) must become a NEGATIVE
    // Unreal yaw, not a positive one, because of the axis flip.
    REQUIRE(thalassa::bridge::facing_yaw_degrees(Vec3{0.0, 1.0, 0.0}) == Approx(-90.0).margin(1e-4));
}

TEST_CASE("facing_yaw_degrees accounts for the handedness flip on -Y (Thalassa right)", "[coordinate_conversion]") {
    REQUIRE(thalassa::bridge::facing_yaw_degrees(Vec3{0.0, -1.0, 0.0}) == Approx(90.0).margin(1e-4));
}

TEST_CASE("facing_yaw_degrees handles facing backward", "[coordinate_conversion]") {
    const double yaw = thalassa::bridge::facing_yaw_degrees(Vec3{-1.0, 0.0, 0.0});
    REQUIRE((yaw == Approx(180.0).margin(1e-4) || yaw == Approx(-180.0).margin(1e-4)));
}

TEST_CASE("facing_yaw_degrees returns 0 for a zero-length direction rather than NaN", "[coordinate_conversion]") {
    REQUIRE(thalassa::bridge::facing_yaw_degrees(Vec3{0.0, 0.0, 0.0}) == 0.0);
}

TEST_CASE("facing_yaw_degrees is independent of direction magnitude (only the angle matters)",
          "[coordinate_conversion]") {
    const double yaw_short = thalassa::bridge::facing_yaw_degrees(Vec3{1.0, 1.0, 0.0});
    const double yaw_long = thalassa::bridge::facing_yaw_degrees(Vec3{50.0, 50.0, 0.0});
    REQUIRE(yaw_short == Approx(yaw_long).margin(1e-6));
}
