#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "thalassa/core/ecs/entity.hpp"
#include "thalassa/core/math/vector.hpp"
#include "thalassa/core/types.hpp"

namespace thalassa::core::ecs {
class World;  // forward declaration: apply_commands only needs a reference/pointer here
}  // namespace thalassa::core::ecs

// "command buffer pattern (input -> commands)" deliverable:
// gameplay-affecting input is expressed as a Command, never applied to the
// World directly by caller code. This is what makes "full replay of a
// fight from recorded commands" possible (see CommandLog below and
// tests/thalassa_sim/command_tests.cpp) — a match is fully reconstructible
// from (rng seed, tick_duration_seconds, the sequence of Commands and
// which tick each was applied on), nothing else, which is also exactly
// the shape networking needs: a client's input becomes a
// Command, the server decides which tick it's authoritative for, and
// applying the same CommandLog to a fresh SimWorld reproduces the match.
//
// Command is a flat, trivially-copyable struct with a type tag rather
// than a tagged union/std::variant of distinct per-command-type structs.
// That trades a few always-present-but-sometimes-unused fields for: (a)
// staying trivially copyable, matching every other wire-shaped type in
// this project (see components.hpp), which keeps the door open to
// reusing ByteWriter::write_pod_array for a future networked command
// stream without bespoke variant serialization, and (b) a simpler,
// single concrete type at every call site (no std::visit boilerplate) for
// a command set that's still small. Revisit as a tagged union if the
// command set grows large enough that the wasted fields become a real
// size/clarity problem.

namespace thalassa::sim {

struct Command {
    enum class Type : std::uint8_t {
        SpawnUnit,
        MoveTo,
    };

    Type type = Type::MoveTo;

    // MoveTo: which entity to move. Ignored for SpawnUnit — the caller
    // can't know a not-yet-created entity's id in advance; apply_commands()
    // returns created entities instead (see below).
    core::ecs::Entity entity{};

    // SpawnUnit: initial position. MoveTo: destination.
    core::math::Vec3 position{0.0, 0.0, 0.0};

    // MoveTo: movement speed, units/second. Unused for SpawnUnit — units
    // always spawn stationary; a separate MoveTo command sets them moving,
    // exactly like a real move order issued after a unit already exists.
    core::Scalar speed = 5.0;

    // SpawnUnit only, below this line.
    core::PlayerId owner{};
    std::int32_t team = 0;
    core::Scalar max_health = 100.0;
    core::Scalar attack_damage = 10.0;
    core::Scalar attack_range = 15.0;
    core::Scalar attack_cooldown_seconds = 1.0;

    [[nodiscard]] static Command spawn_unit(core::math::Vec3 spawn_position, core::PlayerId owner_player,
                                             std::int32_t owner_team, core::Scalar max_health,
                                             core::Scalar attack_damage, core::Scalar attack_range,
                                             core::Scalar attack_cooldown_seconds) {
        Command c;
        c.type = Type::SpawnUnit;
        c.position = spawn_position;
        c.owner = owner_player;
        c.team = owner_team;
        c.max_health = max_health;
        c.attack_damage = attack_damage;
        c.attack_range = attack_range;
        c.attack_cooldown_seconds = attack_cooldown_seconds;
        return c;
    }

    [[nodiscard]] static Command move_to(core::ecs::Entity moving_entity, core::math::Vec3 destination,
                                          core::Scalar move_speed) {
        Command c;
        c.type = Type::MoveTo;
        c.entity = moving_entity;
        c.position = destination;
        c.speed = move_speed;
        return c;
    }
};

// Applies `commands` to `world` in order. Returns the Entity created for
// each SpawnUnit command, in the same relative order those commands
// appeared in `commands` — non-SpawnUnit commands contribute nothing to
// the returned list (not even a placeholder), so callers that issued N
// spawn commands read the first N entries positionally against just their
// spawn commands, not against `commands` as a whole.
[[nodiscard]] std::vector<core::ecs::Entity> apply_commands(core::ecs::World& world,
                                                              const std::vector<Command>& commands);

// CommandLog: records which Commands were applied on which simulation
// tick, so a match can be reconstructed by replaying them against a fresh
// SimWorld started from the same seed — see the file-level comment above.
class CommandLog {
public:
    void add(std::uint64_t tick, Command command) { by_tick_[tick].push_back(command); }

    // Returns the commands recorded for `tick`, in add() call order for
    // that tick, or nullptr if none were recorded. Looking up one
    // specific, known tick key never depends on unordered_map's iteration
    // order — only the *returned vector's* order does, and that's
    // insertion order, controlled entirely by the caller. Same "index by
    // explicit key, never iterate the container" pattern as UniformGrid
    [[nodiscard]] const std::vector<Command>* commands_at(std::uint64_t tick) const {
        const auto it = by_tick_.find(tick);
        return it == by_tick_.end() ? nullptr : &it->second;
    }

    [[nodiscard]] bool empty() const noexcept { return by_tick_.empty(); }
    [[nodiscard]] std::size_t tick_count() const noexcept { return by_tick_.size(); }

private:
    std::unordered_map<std::uint64_t, std::vector<Command>> by_tick_;
};

}  // namespace thalassa::sim
