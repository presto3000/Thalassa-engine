#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "thalassa/bridge/command_builders.hpp"

using Catch::Approx;
using thalassa::bridge::build_move_command;
using thalassa::bridge::build_spawn_command;
using thalassa::bridge::pack_entity;
using thalassa::bridge::UnrealVector;
using thalassa::core::ecs::Entity;
using thalassa::sim::Command;

TEST_CASE("build_spawn_command converts position from Unreal to Thalassa coordinates", "[command_builders]") {
    const UnrealVector unreal_pos{1000.0, -400.0, 200.0};  // see coordinate_conversion_tests.cpp's known example
    const Command cmd =
        build_spawn_command(unreal_pos, thalassa::core::PlayerId{1}, 0, 100.0, 10.0, 1500.0, 1.0);

    REQUIRE(cmd.type == Command::Type::SpawnUnit);
    REQUIRE(cmd.position.x == Approx(10.0));
    REQUIRE(cmd.position.y == Approx(4.0));
    REQUIRE(cmd.position.z == Approx(2.0));
}

TEST_CASE("build_spawn_command converts attack_range from Unreal units to meters", "[command_builders]") {
    const Command cmd = build_spawn_command({0.0, 0.0, 0.0}, {}, 0, 100.0, 10.0, /*range=*/1500.0, 1.0);
    REQUIRE(cmd.attack_range == Approx(15.0));
}

TEST_CASE("build_spawn_command passes health/damage through unscaled (not distances)", "[command_builders]") {
    const Command cmd = build_spawn_command({0.0, 0.0, 0.0}, {}, 0, /*health=*/250.0, /*damage=*/42.0, 500.0, 1.0);
    REQUIRE(cmd.max_health == Approx(250.0));
    REQUIRE(cmd.attack_damage == Approx(42.0));
}

TEST_CASE("build_spawn_command carries owner and team through", "[command_builders]") {
    const Command cmd = build_spawn_command({0.0, 0.0, 0.0}, thalassa::core::PlayerId{99}, 3, 100.0, 10.0, 500.0, 1.0);
    REQUIRE(cmd.owner == thalassa::core::PlayerId{99});
    REQUIRE(cmd.team == 3);
}

TEST_CASE("build_move_command converts entity handle and destination", "[command_builders]") {
    const Entity target{5, 2};
    const Command cmd = build_move_command(pack_entity(target), {200.0, 0.0, 0.0}, 800.0);

    REQUIRE(cmd.type == Command::Type::MoveTo);
    REQUIRE(cmd.entity == target);
    REQUIRE(cmd.position.x == Approx(2.0));
    REQUIRE(cmd.speed == Approx(8.0));  // 800 unreal units/sec -> 8 m/s
}

TEST_CASE("build_hero_spawn_command sets melee/ranged attack profile and converts projectile speed",
          "[command_builders]") {
    const UnrealVector pos{0.0, 0.0, 0.0};
    const Command melee = thalassa::bridge::build_hero_spawn_command(pos, thalassa::core::PlayerId{1}, 0, 100.0,
                                                                     10.0, 200.0, 1.0, true, 0.3, 0.0);
    REQUIRE(melee.attack_kind == 0);
    REQUIRE(melee.attack_windup_seconds == Approx(0.3));
    REQUIRE(melee.attack_range == Approx(2.0));

    const Command ranged = thalassa::bridge::build_hero_spawn_command(pos, thalassa::core::PlayerId{1}, 1, 100.0,
                                                                      10.0, 1500.0, 1.0, false, 0.25, 3000.0);
    REQUIRE(ranged.attack_kind == 1);
    REQUIRE(ranged.projectile_speed == Approx(30.0));  // 3000 uu/s = 30 m/s
    REQUIRE(ranged.attack_range == Approx(15.0));
}
