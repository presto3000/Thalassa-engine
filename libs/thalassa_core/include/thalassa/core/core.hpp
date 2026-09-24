#pragma once

// Convenience umbrella header pulling in the common pieces of
// thalassa_core. Hot-path translation units are still encouraged to
// include only what they need to keep build times down as the project
// grows; this header is aimed at application/glue code (apps/, tests/).

#include "thalassa/core/assert.hpp"
#include "thalassa/core/config.hpp"
#include "thalassa/core/ecs/component_pool.hpp"
#include "thalassa/core/ecs/entity.hpp"
#include "thalassa/core/ecs/world.hpp"
#include "thalassa/core/log.hpp"
#include "thalassa/core/math/strictfp.hpp"
#include "thalassa/core/math/vector.hpp"
#include "thalassa/core/memory/arena.hpp"
#include "thalassa/core/random.hpp"
#include "thalassa/core/serialization/byte_stream.hpp"
#include "thalassa/core/spatial/uniform_grid.hpp"
#include "thalassa/core/time/fixed_timestep.hpp"
#include "thalassa/core/types.hpp"
