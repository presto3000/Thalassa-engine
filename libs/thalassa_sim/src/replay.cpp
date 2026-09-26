#include "thalassa/sim/replay.hpp"

#include <type_traits>

#include "thalassa/core/assert.hpp"
#include "thalassa/sim/components.hpp"

namespace thalassa::sim {

using core::serialization::ByteReader;
using core::serialization::ByteWriter;
using components::CombatStats;
using components::Destination;
using components::Health;
using components::Owner;
using components::Position;
using components::Projectile;
using components::Team;
using components::Velocity;

void save_world(const SimWorld& world, ByteWriter& writer) {
    writer.write_u32(kSnapshotMagic);
    writer.write_u32(kSnapshotVersion);

    writer.write_u64(world.ticks_simulated());
    writer.write_u64(world.rng().state());
    writer.write_u64(world.rng().inc());

    world.world().save_entity_allocator(writer);

    // Fixed, explicit list of serializable component types (see file-level
    // comment in replay.hpp for why this list lives here, not in
    // thalassa_core). find_pool<T>() returning nullptr (a component type
    // never used in this World) is written as an explicit empty pool
    // rather than skipped, so save/load stays a fixed-shape format that
    // doesn't need presence flags per type.
    auto save_component = [&writer](const auto* pool) {
        static const std::remove_reference_t<decltype(*pool)> kEmpty{};
        (pool != nullptr ? *pool : kEmpty).serialize(writer);
    };
    save_component(world.world().find_pool<Position>());
    save_component(world.world().find_pool<Velocity>());
    save_component(world.world().find_pool<Destination>());
    //additions (bumped kSnapshotVersion to 2 — see replay.hpp):
    save_component(world.world().find_pool<Team>());
    save_component(world.world().find_pool<Owner>());
    save_component(world.world().find_pool<Health>());
    save_component(world.world().find_pool<CombatStats>());
    save_component(world.world().find_pool<Projectile>());
}

void load_world(SimWorld& world, ByteReader& reader) {
    const auto magic = reader.read_u32();
    THALASSA_ASSERT_MSG(magic == kSnapshotMagic, "load_world: bad snapshot magic (not a Thalassa snapshot?)");
    const auto version = reader.read_u32();
    THALASSA_ASSERT_MSG(version == kSnapshotVersion, "load_world: unsupported snapshot version");

    const auto ticks = reader.read_u64();
    const auto rng_state = reader.read_u64();
    const auto rng_inc = reader.read_u64();

    world.world().load_entity_allocator(reader);

    world.world().ensure_pool<Position>().deserialize(reader);
    world.world().ensure_pool<Velocity>().deserialize(reader);
    world.world().ensure_pool<Destination>().deserialize(reader);
    world.world().ensure_pool<Team>().deserialize(reader);
    world.world().ensure_pool<Owner>().deserialize(reader);
    world.world().ensure_pool<Health>().deserialize(reader);
    world.world().ensure_pool<CombatStats>().deserialize(reader);
    world.world().ensure_pool<Projectile>().deserialize(reader);

    world.rng().restore(rng_state, rng_inc);
    world.set_ticks_simulated(ticks);
    world.rebuild_spatial_index();
}

}  // namespace thalassa::sim
