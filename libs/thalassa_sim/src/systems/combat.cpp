#include "thalassa/sim/systems/combat.hpp"

#include <limits>
#include <vector>

#include "thalassa/core/math/vector.hpp"
#include "thalassa/sim/components.hpp"

namespace thalassa::sim::systems {

using core::math::Vec3;
using core::ecs::Entity;
using core::ecs::World;
using components::CombatStats;
using components::Health;
using components::Position;
using components::Projectile;
using components::Team;
using components::Velocity;

namespace {

// Projectile flight speed and lifetime are fixed constants
// rather than per-unit configuration — every unit's basic attack fires the
// same kind of projectile. Making these per-CombatStats fields is a small,
// natural extension once gameplay needs different projectile types
// (revisit alongside a real ability system)
constexpr core::Scalar kProjectileSpeed = 40.0;       // units/second
constexpr core::Scalar kProjectileTtlSeconds = 3.0;   // auto-expire if it never hits
constexpr core::Scalar kProjectileHitRadius = 1.0;    // units

struct PendingProjectile {
    Vec3 origin;
    Vec3 direction;
    core::Scalar damage;
    std::int32_t owner_team;
};

}  // namespace

void combat_system(World& world, const SpatialIndex& spatial_index, core::Scalar dt_seconds) {
    std::vector<PendingProjectile> pending;

    world.each<Position, Team, CombatStats>(
        [&world, &spatial_index, &pending, dt_seconds](Entity e, Position& pos, Team& team, CombatStats& stats) {
            stats.cooldown_remaining_seconds -= dt_seconds;
            if (stats.cooldown_remaining_seconds > 0.0) {
                return;
            }
            stats.cooldown_remaining_seconds = 0.0;

            std::vector<Entity> candidates;
            spatial_index.query_radius(pos.value, stats.attack_range, candidates);

            Entity best_target{};
            core::Scalar best_dist_sq = std::numeric_limits<core::Scalar>::max();
            for (const Entity candidate : candidates) {
                if (candidate == e) {
                    continue;
                }
                if (!world.has<Team>(candidate) || !world.has<Health>(candidate)) {
                    continue;  // not a combat-capable entity (e.g. another projectile)
                }
                if (world.get<Team>(candidate).id == team.id) {
                    continue;  // not an enemy
                }
                if (world.get<Health>(candidate).current <= 0.0) {
                    continue;  // already dead this tick, about to be removed by death_system
                }

                const core::Scalar dist_sq = (world.get<Position>(candidate).value - pos.value).length_sq();
                // Tie-break by lowest index for a fully deterministic pick
                // even on an exact distance tie 
                if (dist_sq < best_dist_sq ||
                    (dist_sq == best_dist_sq && (!best_target.is_valid() || candidate.index() < best_target.index()))) {
                    best_dist_sq = dist_sq;
                    best_target = candidate;
                }
            }

            if (!best_target.is_valid()) {
                return;  // no target in range: stay ready to fire next tick, don't reset cooldown
            }

            stats.cooldown_remaining_seconds = stats.attack_cooldown_seconds;

            const Vec3 target_pos = world.get<Position>(best_target).value;
            const Vec3 direction = (target_pos - pos.value).normalized();
            pending.push_back(PendingProjectile{pos.value, direction, stats.attack_damage, team.id});
        });

    // Spawning happens strictly after the query loop above completes — design, not a style choice.
    for (const PendingProjectile& p : pending) {
        const Entity projectile = world.create_entity();
        world.emplace<Position>(projectile, Position{p.origin});
        world.emplace<Velocity>(projectile, Velocity{p.direction * kProjectileSpeed});
        world.emplace<Projectile>(projectile, Projectile{p.damage, p.owner_team, kProjectileTtlSeconds});
    }
}

void projectile_collision_system(World& world, const SpatialIndex& spatial_index, core::Scalar dt_seconds) {
    std::vector<Entity> to_destroy;

    world.each<Position, Projectile>(
        [&world, &spatial_index, &to_destroy, dt_seconds](Entity e, Position& pos, Projectile& projectile) {
            std::vector<Entity> candidates;
            spatial_index.query_radius(pos.value, kProjectileHitRadius, candidates);

            Entity hit{};
            for (const Entity candidate : candidates) {
                if (candidate == e) {
                    continue;
                }
                if (!world.has<Team>(candidate) || !world.has<Health>(candidate)) {
                    continue;
                }
                if (world.get<Team>(candidate).id == projectile.owner_team) {
                    continue;
                }
                if (world.get<Health>(candidate).current <= 0.0) {
                    continue;
                }
                hit = candidate;
                break;  // first match in deterministic grid-traversal order
            }

            if (hit.is_valid()) {
                world.get<Health>(hit).current -= projectile.damage;
                to_destroy.push_back(e);
                return;
            }

            projectile.ttl_seconds -= dt_seconds;
            if (projectile.ttl_seconds <= 0.0) {
                to_destroy.push_back(e);
            }
        });

    for (const Entity e : to_destroy) {
        world.destroy_entity(e);
    }
}

void death_system(World& world) {
    std::vector<Entity> to_destroy;

    // Collected first, destroyed after: each<Health> (a single-component
    // query) iterates ComponentPool<Health>'s live dense arrays directly
    // rather than a snapshot copy (unlike the 2- and 3-component
    // overloads), so destroying an entity mid-callback here would shrink
    // the very array being iterated
    world.each<Health>([&to_destroy](Entity e, const Health& health) {
        if (health.current <= 0.0) {
            to_destroy.push_back(e);
        }
    });

    for (const Entity e : to_destroy) {
        world.destroy_entity(e);
    }
}

}  // namespace thalassa::sim::systems
