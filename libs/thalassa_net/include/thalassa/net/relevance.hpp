#pragma once

#include <functional>

#include "thalassa/core/ecs/world.hpp"
#include "thalassa/core/types.hpp"

// Entity relevance: which entities get sent to which viewer.

namespace thalassa::net {

// Returns true if `entity` should be included in the data sent to
// `viewer`. Takes the World so real policies can inspect position/team/
// ownership; `always_relevant` below ignores all of that.
using RelevanceFilter =
    std::function<bool(const core::ecs::World& world, core::ecs::Entity entity, core::PlayerId viewer)>;

namespace relevance {

// every entity is relevant to every viewer (full
// visibility, no fog of war)
[[nodiscard]] inline bool always_relevant(const core::ecs::World& /*world*/, core::ecs::Entity /*entity*/,
                                           core::PlayerId /*viewer*/) {
    return true;
}

}  // namespace relevance

}  // namespace thalassa::net
