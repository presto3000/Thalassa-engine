#pragma once

#include <vector>

#include "thalassa/core/ecs/entity.hpp"
#include "thalassa/core/math/vector.hpp"
#include "thalassa/core/types.hpp"
#include "thalassa/sim/commands.hpp"

namespace thalassa::core::ecs {
class World;
}

// Group moves (pathfinding step 4b): when a selection is sent to ONE point, give every unit its own
// goal around it instead of piling them onto the same spot (separation would otherwise have to
// untangle them, and units that arrive late would stall behind the early ones).
//
//   spread_slots(center, count, spacing): `count` distinct points: the center, then hexagonal rings
//   (6, 12, 18, ... points at radius spacing, 2*spacing, ...). Deterministic, pure.
//
//   plan_group_move(world, units, center, speed, spacing): one MoveTo per living unit. The closest
//   unit to the center takes the center, then the next closest takes the next slot, and so on
//   (ties by entity index), so units end up roughly where they came from, crossing as little as a
//   cheap rule can manage. Read-only on the world; the caller submits the commands (so they go
//   through the normal command log / network path). `spacing` should be at least 2 x the unit radius.

namespace thalassa::sim {

[[nodiscard]] std::vector<core::math::Vec3> spread_slots(const core::math::Vec3& center, std::size_t count,
                                                          core::Scalar spacing);

[[nodiscard]] std::vector<Command> plan_group_move(const core::ecs::World& world,
                                                    const std::vector<core::ecs::Entity>& units,
                                                    const core::math::Vec3& center, core::Scalar speed,
                                                    core::Scalar spacing);

}  // namespace thalassa::sim
