#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "thalassa/core/math/vector.hpp"

using Catch::Approx;
using namespace thalassa::core::math;

TEST_CASE("Vec3 basic arithmetic", "[math]") {
    const Vec3 a{1.0, 2.0, 3.0};
    const Vec3 b{4.0, 5.0, 6.0};

    REQUIRE((a + b) == Vec3{5.0, 7.0, 9.0});
    REQUIRE((b - a) == Vec3{3.0, 3.0, 3.0});
    REQUIRE((a * 2.0) == Vec3{2.0, 4.0, 6.0});
    REQUIRE(a.dot(b) == 32.0);
}

TEST_CASE("Vec3 cross product is perpendicular to both inputs", "[math]") {
    const Vec3 x{1.0, 0.0, 0.0};
    const Vec3 y{0.0, 1.0, 0.0};
    const Vec3 z = x.cross(y);
    REQUIRE(z == Vec3{0.0, 0.0, 1.0});
}

TEST_CASE("Vec3 normalized has unit length", "[math]") {
    const Vec3 v{3.0, 4.0, 0.0};
    const Vec3 n = v.normalized();
    REQUIRE(n.length() == Approx(1.0).epsilon(1e-9));
    REQUIRE(n.x == Approx(0.6).epsilon(1e-9));
    REQUIRE(n.y == Approx(0.8).epsilon(1e-9));
}

TEST_CASE("Vec3 normalized of zero vector returns zero, not NaN", "[math]") {
    const Vec3 v{0.0, 0.0, 0.0};
    const Vec3 n = v.normalized();
    REQUIRE(n.x == 0.0);
    REQUIRE(n.y == 0.0);
    REQUIRE(n.z == 0.0);
}

TEST_CASE("Quat identity has zero rotation effect on length", "[math]") {
    const Quat q;
    REQUIRE(q.length() == Approx(1.0).epsilon(1e-9));
}

TEST_CASE("Quat from_axis_angle around Z by 90 degrees is normalized", "[math]") {
    const Quat q = Quat::from_axis_angle(Vec3{0.0, 0.0, 1.0}, strictfp::kHalfPi);
    REQUIRE(q.length() == Approx(1.0).epsilon(1e-9));
}
