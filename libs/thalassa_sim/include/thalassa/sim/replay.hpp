#pragma once

#include "thalassa/core/serialization/byte_stream.hpp"
#include "thalassa/sim/sim_world.hpp"

// World-state snapshotting for SimWorld, built on thalassa_core's
// ByteWriter/ByteReader and ComponentPool<T>::serialize/deserialize. This
// is "serialization of world state (for save/replay
// later)" deliverable: a snapshot captures everything needed to resume an
// identical simulation (entity allocator state, RNG state, tick counter,
// and every registered component pool), and reloading one and continuing
// to simulate produces bit-identical results to an uninterrupted run — see
// tests/thalassa_sim/replay_tests.cpp for that proof.
//
// This is deliberately NOT a full replay system yet (that needs the
// command-buffer pattern: record *inputs*, not just
// periodic full-state snapshots, so a replay is "resimulate from initial
// state + recorded commands" rather than "store every tick's full
// state"). What's here is the save/load primitive that milestone will
// build on, plus it's independently useful right now as a save-game /
// server-resync mechanism.
//
// Component-type registration: World's serialization support
// (ensure_pool<T>()/find_pool<T>()) is generic over T, but something has
// to know the concrete, fixed list of component types this project
// actually uses — that's here, not in thalassa_core, because thalassa_core
// must stay gameplay-agnostic (see the module-boundary rule in the root
// CMakeLists.txt comments and docs/ecs_decision.md). Adding a new
// serializable component type later, means adding one line
// to save_world()/load_world()'s call sequence — nothing in thalassa_core
// changes.

namespace thalassa::sim {

inline constexpr std::uint32_t kSnapshotMagic = 0x54484C41;  // "THLA" (Thalassa)
inline constexpr std::uint32_t kSnapshotVersion = 2;  // v2: adds Team/Owner/Health/CombatStats/Projectile

// Serializes `world` into `writer`. See the file-level comment for scope.
void save_world(const SimWorld& world, core::serialization::ByteWriter& writer);

// Replaces the contents of `world` with what `reader` holds. `world`
// should be freshly constructed (any existing entities/components are
// discarded — see World::load_entity_allocator). The RNG seed originally
// used to construct `world` is irrelevant after this call: RNG *state* is
// restored from the snapshot, overriding whatever the constructor seeded.
void load_world(SimWorld& world, core::serialization::ByteReader& reader);

}  // namespace thalassa::sim
