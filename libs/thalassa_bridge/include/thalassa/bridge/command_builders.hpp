#pragma once

#include "thalassa/bridge/coordinate_conversion.hpp"
#include "thalassa/bridge/entity_handle.hpp"
#include "thalassa/core/types.hpp"
#include "thalassa/sim/commands.hpp"

// Builds thalassa::sim::Command values from Unreal-side parameters,
// performing the coordinate conversion. This is the exact logic
// UThalassaWorldSubsystem::SpawnUnit()/MoveUnitTo() call into — pulled out
// here so it's unit-tested against real inputs rather than only ever
// exercised from inside UE5-specific code this environment can't compile.
//
// Takes UnrealVector/plain scalars (not FVector) for the same reason
// coordinate_conversion.hpp does: no Unreal types appear in this header at
// all, so it's part of the compiled/tested surface, not the unverified
// part

namespace thalassa::bridge {

[[nodiscard]] inline sim::Command build_spawn_command(UnrealVector unreal_spawn_position, core::PlayerId owner,
                                                        std::int32_t team, double unreal_max_health,
                                                        double unreal_attack_damage, double unreal_attack_range,
                                                        core::Scalar attack_cooldown_seconds) {
    // Health and damage are plain numbers, not distances — they don't go
    // through the meters<->centimeters scale at all, only attack_range
    // does (it's a distance). Taking `double` for all of these (rather
    // than core::Scalar) keeps this function's signature "what a UE5 call
    // site naturally has on hand" (Unreal's own reflection-friendly
    // numeric type is float/double, not a project-specific Scalar alias).
    return sim::Command::spawn_unit(to_thalassa(unreal_spawn_position), owner, team,
                                     static_cast<core::Scalar>(unreal_max_health),
                                     static_cast<core::Scalar>(unreal_attack_damage),
                                     unreal_units_to_meters(unreal_attack_range), attack_cooldown_seconds);
}

[[nodiscard]] inline sim::Command build_move_command(PackedEntityHandle entity, UnrealVector unreal_destination,
                                                       double unreal_speed) {
    return sim::Command::move_to(unpack_entity(entity), to_thalassa(unreal_destination),
                                  unreal_units_to_meters(unreal_speed));
}

}  // namespace thalassa::bridge
