#pragma once

#include "thalassa/core/serialization/byte_stream.hpp"
#include "thalassa/core/types.hpp"
#include "thalassa/net/relevance.hpp"
#include "thalassa/sim/sim_world.hpp"

// SnapshotDelta: "snapshot system + delta compression"
// deliverable. Encodes the *difference* between two SimWorld states —
// `baseline` (a state the viewer is already known to have, e.g. from a
// prior full snapshot or an earlier delta) and `current` (the state to
// bring them up to date with) — as a much smaller payload than a full
// snapshot when few entities actually changed, filtered through a
// RelevanceFilter so only entities the viewer should see are included at
// all.
//
// Typical usage: a client receives one full snapshot (thalassa::sim::
// save_world/load_world) to establish a baseline, then a stream of deltas
// each relative to the previous one. apply_delta expects `world` to
// already be in exactly the `baseline` state this delta was computed
// against — applying a delta to the wrong baseline produces wrong (though
// not undefined-behavior-unsafe) results, same as any delta-compression
// scheme.
//
// Format (all fields in this order):
//   u64 destroyed_entity_count, then that many Entity (index+generation)
//     — entities alive+relevant in `baseline` that are no longer
//       alive+relevant in `current`, replicated as real destruction
//       (see World::for_each_alive_entity / destroy_entity on apply), not
//       merely "lost every component."
//   For each registered component type, in the same fixed order
//   replay.hpp's save_world/load_world uses (Position, Velocity,
//   Destination, Team, Owner, Health, CombatStats, Projectile):
//     [upserts]  u64 count, that many Entity, then that many T (POD block)
//     [removals] u64 count, that many Entity
//       — "has this component removed without the whole entity being
//         destroyed" OR "no longer relevant to this viewer" (from the
//         viewer's perspective these look the same: the entity should
//         disappear from their view of this component).

namespace thalassa::net {

void save_delta(const sim::SimWorld& baseline, const sim::SimWorld& current, core::PlayerId viewer,
                 const RelevanceFilter& filter, core::serialization::ByteWriter& writer);

// Applies a delta to `world`, which must already be in the exact state
// `baseline` was in when save_delta() produced this buffer (see file-level
// comment above).
void apply_delta(sim::SimWorld& world, core::serialization::ByteReader& reader);

}  // namespace thalassa::net
