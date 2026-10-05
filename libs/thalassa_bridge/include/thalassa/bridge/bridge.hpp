#pragma once

// thalassa_bridge: engine-agnostic logic supporting the UE5 integration
// (unreal/ThalassaPlugin/) — coordinate/unit conversion, entity handle
// packing, and command-building.Deliberately contains no
// Unreal Engine headers or types, so it's built and tested by this
// repo's normal CMake/Catch2 toolchain like every other module.

#include "thalassa/bridge/command_builders.hpp"
#include "thalassa/bridge/coordinate_conversion.hpp"
#include "thalassa/bridge/entity_handle.hpp"
