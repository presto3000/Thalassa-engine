#include "thalassa/sim/systems/combat.hpp"

#include <limits>
#include <vector>

#include "thalassa/core/math/vector.hpp"
#include "thalassa/sim/components.hpp"

namespace thalassa::sim::systems {

using core::math::Vec3;
using core::ecs::Entity;
using core::ecs::World;
using components::AttackProfile;
using components::AttackState;
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
// (revisit alongside a real ability system — see
// docs/milestone2_combat_design.md).
constexpr core::Scalar kProjectileSpeed = 40.0;       // units/second
constexpr core::Scalar kProjectileTtlSeconds = 3.0;   // auto-expire if it never hits
constexpr core::Scalar kProjectileHitRadius = 1.0;    // units

// Deferred spawn request — see docs/milestone2_combat_design.md ("Why
// entity creation/destruction never happens mid-each<...>-callback") for
// why combat_system builds a list of these instead of calling
// world.create_entity() from inside its each<Position, Team, CombatStats>
// callback.
struct PendingProjectile {
    Vec3 origin;
    Vec3 direction;
    core::Scalar damage;
    std::int32_t owner_team;
    core::Scalar speed;
    Entity source;
    Entity target;
};

// Melee release: damage is applied after the query loop, in attacker order,
// for the same reason spawns are deferred (a later attacker in the same tick
// must not see a half-updated world mid-iteration).
struct PendingMelee {
    Entity source;
    Entity target;
    core::Scalar damage;
};

}  // namespace

void combat_system(World& world, const SpatialIndex& spatial_index, core::Scalar dt_seconds, EventBuffer* events) {
    std::vector<PendingProjectile> pending;
    std::vector<PendingMelee> pending_melee;

    // Nearest living enemy within attack_range, ties -> lowest Entity::index
    // (fully deterministic even on an exact distance tie — see
    // docs/milestone2_combat_design.md).
    auto find_target = [&world, &spatial_index](Entity self, const Position& pos, const Team& team,
                                                const CombatStats& stats) {
        std::vector<Entity> candidates;
        spatial_index.query_radius(pos.value, stats.attack_range, candidates);

        Entity best_target{};
        core::Scalar best_dist_sq = std::numeric_limits<core::Scalar>::max();
        for (const Entity candidate : candidates) {
            if (candidate == self) {
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
            if (dist_sq < best_dist_sq ||
                (dist_sq == best_dist_sq && (!best_target.is_valid() || candidate.index() < best_target.index()))) {
                best_dist_sq = dist_sq;
                best_target = candidate;
            }
        }
        return best_target;
    };

    world.each<Position, Team, CombatStats>([&](Entity e, Position& pos, Team& team, CombatStats& stats) {
        stats.cooldown_remaining_seconds -= dt_seconds;

        // ---- Windup/release flow (heroes with an AttackProfile) ----------
        if (world.has<AttackProfile>(e) && world.has<AttackState>(e)) {
            const AttackProfile& profile = world.get<AttackProfile>(e);
            AttackState& state = world.get<AttackState>(e);

            // Releases the attack at the current tick: melee = deferred
            // direct damage, ranged = deferred projectile.
            auto release = [&]() {
                const Vec3 target_pos = world.get<Position>(state.target).value;
                if (profile.kind == AttackProfile::kMelee) {
                    pending_melee.push_back(PendingMelee{e, state.target, stats.attack_damage});
                } else {
                    pending.push_back(PendingProjectile{pos.value, (target_pos - pos.value).normalized(),
                                                        stats.attack_damage, team.id, profile.projectile_speed, e,
                                                        state.target});
                }
                state.phase = AttackState::kIdle;
                state.target = Entity{};
                state.windup_remaining_seconds = 0.0;
            };

            if (state.phase == AttackState::kWindup) {
                const bool target_ok =
                    world.is_alive(state.target) && world.has<Health>(state.target) &&
                    world.has<Position>(state.target) && world.get<Health>(state.target).current > 0.0 &&
                    (world.get<Position>(state.target).value - pos.value).length_sq() <=
                        stats.attack_range * stats.attack_range;
                if (!target_ok) {
                    if (events != nullptr) {
                        events->emit(SimEvent{SimEvent::Type::AttackCancelled, 0, e, state.target, Entity{}, pos.value, 0.0});
                    }
                    state.phase = AttackState::kIdle;
                    state.target = Entity{};
                    state.windup_remaining_seconds = 0.0;
                    stats.cooldown_remaining_seconds = 0.0;  // interrupted attack costs no cooldown
                    return;
                }
                state.windup_remaining_seconds -= dt_seconds;
                if (state.windup_remaining_seconds <= 0.0) {
                    release();
                }
                return;
            }

            // Idle: start a new attack when ready.
            if (stats.cooldown_remaining_seconds > 0.0) {
                return;
            }
            stats.cooldown_remaining_seconds = 0.0;

            const Entity target = find_target(e, pos, team, stats);
            if (!target.is_valid()) {
                return;
            }
            // Cooldown counts from the START of the attack (like Dota's attack
            // rate), so backswing = cooldown - windup needs no simulation.
            stats.cooldown_remaining_seconds = stats.attack_cooldown_seconds;
            state.phase = AttackState::kWindup;
            state.target = target;
            state.windup_remaining_seconds = profile.windup_seconds;
            if (events != nullptr) {
                events->emit(SimEvent{SimEvent::Type::AttackStarted, 0, e, target, Entity{}, pos.value,
                                      profile.windup_seconds, stats.attack_cooldown_seconds});
            }
            if (state.windup_remaining_seconds <= 0.0) {
                release();  // zero windup: release in the same tick
            }
            return;
        }

        // ---- Legacy flow (no AttackProfile): instant basic projectile ----
        if (stats.cooldown_remaining_seconds > 0.0) {
            return;
        }
        stats.cooldown_remaining_seconds = 0.0;

        const Entity best_target = find_target(e, pos, team, stats);
        if (!best_target.is_valid()) {
            return;  // no target in range: stay ready to fire next tick, don't reset cooldown
        }

        stats.cooldown_remaining_seconds = stats.attack_cooldown_seconds;

        const Vec3 target_pos = world.get<Position>(best_target).value;
        const Vec3 direction = (target_pos - pos.value).normalized();
        pending.push_back(
            PendingProjectile{pos.value, direction, stats.attack_damage, team.id, kProjectileSpeed, e, best_target});
    });

    // Melee damage, in attacker order. A target already at 0 HP (killed by an
    // earlier attacker this tick) is skipped, deterministically.
    for (const PendingMelee& m : pending_melee) {
        if (!world.is_alive(m.target) || !world.has<Health>(m.target)) {
            continue;
        }
        Health& health = world.get<Health>(m.target);
        if (health.current <= 0.0) {
            continue;
        }
        health.current -= m.damage;
        if (events != nullptr) {
            events->emit(SimEvent{SimEvent::Type::MeleeHit, 0, m.source, m.target, Entity{},
                                  world.get<Position>(m.target).value, m.damage});
        }
    }

    // Spawning happens strictly after the query loop above completes — see
    // docs/milestone2_combat_design.md for why this ordering is a
    // correctness requirement, not a style choice.
    for (const PendingProjectile& p : pending) {
        const Entity projectile = world.create_entity();
        world.emplace<Position>(projectile, Position{p.origin});
        world.emplace<Velocity>(projectile, Velocity{p.direction * p.speed});
        world.emplace<Projectile>(projectile, Projectile::make(p.damage, p.owner_team, kProjectileTtlSeconds));
        if (events != nullptr) {
            events->emit(SimEvent{SimEvent::Type::ProjectileLaunched, 0, p.source, p.target, projectile, p.origin,
                                  p.damage});
        }
    }
}

void projectile_collision_system(World& world, const SpatialIndex& spatial_index, core::Scalar dt_seconds,
                                 EventBuffer* events) {
    std::vector<Entity> to_destroy;

    world.each<Position, Projectile>(
        [&world, &spatial_index, &to_destroy, &events, dt_seconds](Entity e, Position& pos, Projectile& projectile) {
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
                if (events != nullptr) {
                    events->emit(SimEvent{SimEvent::Type::ProjectileHit, 0, Entity{}, hit, e, pos.value,
                                          projectile.damage});
                }
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

void death_system(World& world, EventBuffer* events) {
    std::vector<Entity> to_destroy;

    // Collected first, destroyed after: each<Health> (a single-component
    // query) iterates ComponentPool<Health>'s live dense arrays directly
    // rather than a snapshot copy (unlike the 2- and 3-component
    // overloads), so destroying an entity mid-callback here would shrink
    // the very array being iterated — see
    // docs/milestone2_combat_design.md.
    world.each<Health>([&to_destroy](Entity e, const Health& health) {
        if (health.current <= 0.0) {
            to_destroy.push_back(e);
        }
    });

    for (const Entity e : to_destroy) {
        if (events != nullptr) {
            const Vec3 where = world.has<Position>(e) ? world.get<Position>(e).value : Vec3{0.0, 0.0, 0.0};
            events->emit(SimEvent{SimEvent::Type::UnitDied, 0, Entity{}, e, Entity{}, where, 0.0});
        }
        world.destroy_entity(e);
    }
}

}  // namespace thalassa::sim::systems
