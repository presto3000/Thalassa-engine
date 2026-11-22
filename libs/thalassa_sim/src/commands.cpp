#include "thalassa/sim/commands.hpp"

#include "thalassa/core/ecs/world.hpp"
#include "thalassa/sim/components.hpp"

namespace thalassa::sim {

using core::ecs::Entity;
using core::ecs::World;

std::vector<Entity> apply_commands(World& world, const std::vector<Command>& commands) {
    std::vector<Entity> spawned;

    for (const Command& cmd : commands) {
        switch (cmd.type) {
            case Command::Type::SpawnUnit: {
                const Entity e = world.create_entity();
                world.emplace<components::Position>(e, components::Position{cmd.position});
                world.emplace<components::Owner>(e, components::Owner{cmd.owner});
                world.emplace<components::Team>(e, components::Team{cmd.team});
                world.emplace<components::Health>(e, components::Health{cmd.max_health, cmd.max_health});
                world.emplace<components::CombatStats>(
                    e, components::CombatStats{cmd.attack_damage, cmd.attack_range, cmd.attack_cooldown_seconds, 0.0});
                if (cmd.radius > 0.0) world.emplace<components::Radius>(e, components::Radius{cmd.radius});
                if (cmd.attack_kind != Command::kNoProfile) {
                    components::AttackProfile profile;
                    profile.kind = cmd.attack_kind;
                    profile.windup_seconds = cmd.attack_windup_seconds;
                    profile.projectile_speed = cmd.projectile_speed;
                    world.emplace<components::AttackProfile>(e, profile);
                    world.emplace<components::AttackState>(e, components::AttackState{});
                }
                spawned.push_back(e);
                break;
            }
            case Command::Type::MoveTo: {
                // Silently ignored if the entity is no longer alive (e.g.
                // it died in an earlier tick than a stale/late-arriving
                // command references) rather than asserting — a command
                // referencing a since-destroyed entity is an expected
                // occurrence once real (possibly-delayed) player input
                // exists, not a programming error, so it shouldn't abort
                // the whole tick's command application.
                if (world.is_alive(cmd.entity)) {
                    // Fixed 0.5-unit arrival radius: Command doesn't carry
                    // a per-order arrival radius yet (kept minimal per
                    // Milestone 2 scope) — add one if/when gameplay needs
                    // per-order precision control.
                    if (world.has<components::Chase>(cmd.entity)) world.remove<components::Chase>(cmd.entity);
                    world.emplace<components::Destination>(cmd.entity,
                                                             components::Destination{cmd.position, cmd.speed, 0.5});
                }
                break;
            }
            case Command::Type::AttackTarget: {
                // Ignored unless both are alive, the attacker can fight, and it is not itself.
                if (world.is_alive(cmd.entity) && world.is_alive(cmd.target) && cmd.entity != cmd.target &&
                    world.has<components::CombatStats>(cmd.entity) && world.has<components::Position>(cmd.entity) &&
                    world.has<components::Position>(cmd.target)) {
                    if (world.has<components::Destination>(cmd.entity)) world.remove<components::Destination>(cmd.entity);
                    if (world.has<components::Path>(cmd.entity)) world.remove<components::Path>(cmd.entity);
                    components::Chase chase;
                    chase.target = cmd.target;
                    chase.speed = cmd.speed;
                    world.emplace<components::Chase>(cmd.entity, chase);
                }
                break;
            }
        }
    }

    return spawned;
}

}  // namespace thalassa::sim
