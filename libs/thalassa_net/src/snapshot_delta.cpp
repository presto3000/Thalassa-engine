#include "thalassa/net/snapshot_delta.hpp"

#include <cstring>
#include <vector>

#include "thalassa/sim/components.hpp"

namespace thalassa::net {

using core::ecs::Entity;
using core::ecs::World;
using core::serialization::ByteReader;
using core::serialization::ByteWriter;
using sim::SimWorld;

namespace {

template <typename T>
bool is_visible(const World& world, Entity e, core::PlayerId viewer, const RelevanceFilter& filter) {
    return world.has<T>(e) && filter(world, e, viewer);
}

// Diffs one component type's pool between baseline and current, writing
// its upsert/removal blocks. See snapshot_delta.hpp's format comment.
template <typename T>
void save_component_delta(const SimWorld& baseline, const SimWorld& current, core::PlayerId viewer,
                           const RelevanceFilter& filter, ByteWriter& writer) {
    std::vector<Entity> upsert_entities;
    std::vector<T> upsert_values;
    std::vector<Entity> removed_entities;

    if (const auto* current_pool = current.world().find_pool<T>()) {
        const auto& entities = current_pool->entities();
        const auto& values = current_pool->values();
        for (std::size_t i = 0; i < entities.size(); ++i) {
            const Entity e = entities[i];
            if (!filter(current.world(), e, viewer)) {
                continue;  // not visible now; if it *was* visible before, the removal pass below covers it
            }

            const bool baseline_visible = is_visible<T>(baseline.world(), e, viewer, filter);
            bool changed = true;
            if (baseline_visible) {
                const T& baseline_value = baseline.world().get<T>(e);
                changed = std::memcmp(&baseline_value, &values[i], sizeof(T)) != 0;
            }
            if (!baseline_visible || changed) {
                upsert_entities.push_back(e);
                upsert_values.push_back(values[i]);
            }
        }
    }

    if (const auto* baseline_pool = baseline.world().find_pool<T>()) {
        const auto& entities = baseline_pool->entities();
        for (const Entity e : entities) {
            if (!filter(baseline.world(), e, viewer)) {
                continue;  // wasn't visible before either; nothing "disappeared" for this viewer
            }
            if (!is_visible<T>(current.world(), e, viewer, filter)) {
                removed_entities.push_back(e);
            }
        }
    }

    writer.write_pod_array<Entity>(std::span<const Entity>(upsert_entities));
    writer.write_pod_array<T>(std::span<const T>(upsert_values));
    writer.write_pod_array<Entity>(std::span<const Entity>(removed_entities));
}

template <typename T>
void apply_component_delta(World& world, ByteReader& reader) {
    const auto upsert_entities = reader.read_pod_array<Entity>();
    const auto upsert_values = reader.read_pod_array<T>();
    THALASSA_ASSERT_MSG(upsert_entities.size() == upsert_values.size(),
                         "apply_component_delta: entity/value count mismatch");
    for (std::size_t i = 0; i < upsert_entities.size(); ++i) {
        world.ensure_entity(upsert_entities[i]);
        world.emplace<T>(upsert_entities[i], upsert_values[i]);
    }

    const auto removed_entities = reader.read_pod_array<Entity>();
    for (const Entity e : removed_entities) {
        if (world.is_alive(e)) {
            world.remove<T>(e);
        }
    }
}

}  // namespace

void save_delta(const SimWorld& baseline, const SimWorld& current, core::PlayerId viewer,
                 const RelevanceFilter& filter, ByteWriter& writer) {
    std::vector<Entity> destroyed_entities;
    baseline.world().for_each_alive_entity([&](Entity e) {
        if (!filter(baseline.world(), e, viewer)) {
            return;  // wasn't visible before; its disappearance isn't news to this viewer
        }
        if (!current.world().is_alive(e) || !filter(current.world(), e, viewer)) {
            destroyed_entities.push_back(e);
        }
    });
    writer.write_pod_array<Entity>(std::span<const Entity>(destroyed_entities));

    save_component_delta<sim::components::Position>(baseline, current, viewer, filter, writer);
    save_component_delta<sim::components::Velocity>(baseline, current, viewer, filter, writer);
    save_component_delta<sim::components::Destination>(baseline, current, viewer, filter, writer);
    save_component_delta<sim::components::Team>(baseline, current, viewer, filter, writer);
    save_component_delta<sim::components::Owner>(baseline, current, viewer, filter, writer);
    save_component_delta<sim::components::Health>(baseline, current, viewer, filter, writer);
    save_component_delta<sim::components::CombatStats>(baseline, current, viewer, filter, writer);
    save_component_delta<sim::components::Projectile>(baseline, current, viewer, filter, writer);
}

void apply_delta(SimWorld& world, ByteReader& reader) {
    const auto destroyed_entities = reader.read_pod_array<Entity>();
    for (const Entity e : destroyed_entities) {
        if (world.world().is_alive(e)) {
            world.world().destroy_entity(e);
        }
    }

    apply_component_delta<sim::components::Position>(world.world(), reader);
    apply_component_delta<sim::components::Velocity>(world.world(), reader);
    apply_component_delta<sim::components::Destination>(world.world(), reader);
    apply_component_delta<sim::components::Team>(world.world(), reader);
    apply_component_delta<sim::components::Owner>(world.world(), reader);
    apply_component_delta<sim::components::Health>(world.world(), reader);
    apply_component_delta<sim::components::CombatStats>(world.world(), reader);
    apply_component_delta<sim::components::Projectile>(world.world(), reader);

    world.rebuild_spatial_index();
}

}  // namespace thalassa::net
