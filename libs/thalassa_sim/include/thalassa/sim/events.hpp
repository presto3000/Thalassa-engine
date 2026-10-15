#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "thalassa/core/ecs/entity.hpp"
#include "thalassa/core/math/vector.hpp"
#include "thalassa/core/types.hpp"

// SimEvent: "something visible just happened" notifications for the
// presentation layer (Unreal animations, VFX, sounds, floating damage
// numbers). Events are OUTPUT ONLY: the simulation never reads them, they
// are not part of snapshots/checksums, and a replay regenerates exactly the
// same events because they come from the same deterministic systems. So
// presentation can never influence gameplay.
//
// Collection is OFF by default (headless servers and tests pay nothing); a
// presentation host turns it on with SimWorld::set_events_enabled(true) and
// drains it every frame with SimWorld::drain_events(). Several ticks may run
// between two frames; events from all of them arrive in order, each stamped
// with the tick it happened on.

namespace thalassa::sim {

struct SimEvent {
    enum class Type : std::uint8_t {
        AttackStarted,       // windup began: play the attack animation. source=attacker, target=locked target,
                             //   value=windup seconds (release moment), value2=attack_cooldown seconds (whole swing)
        AttackCancelled,     // windup interrupted (target died / left range). source=attacker
        MeleeHit,            // melee release: damage applied this tick. source=attacker, target, value=damage
        ProjectileLaunched,  // ranged release: source=attacker, target, projectile=new entity, value=damage
        ProjectileHit,       // projectile reached an enemy. target, projectile, value=damage
        UnitDied,            // target=entity that is about to be removed (position = where it died)
    };

    Type type = Type::AttackStarted;
    std::uint64_t tick = 0;
    core::ecs::Entity source{};
    core::ecs::Entity target{};
    core::ecs::Entity projectile{};
    core::math::Vec3 position{0.0, 0.0, 0.0};  // where it happened (attacker / impact / death)
    core::Scalar value = 0.0;
    core::Scalar value2 = 0.0;  // AttackStarted only: total attack time (cooldown), see above
};

class EventBuffer {
public:
    void set_enabled(bool enabled) noexcept { enabled_ = enabled; }
    [[nodiscard]] bool enabled() const noexcept { return enabled_; }

    void set_current_tick(std::uint64_t tick) noexcept { tick_ = tick; }

    void emit(SimEvent event) {
        if (!enabled_) return;
        if (events_.size() >= kMaxBuffered) { ++dropped_; return; }  // nobody is draining
        event.tick = tick_;
        events_.push_back(event);
    }

    // Returns everything collected since the last drain and empties the buffer.
    [[nodiscard]] std::vector<SimEvent> drain() { return std::exchange(events_, {}); }
    void clear() noexcept { events_.clear(); }

    [[nodiscard]] std::size_t size() const noexcept { return events_.size(); }
    [[nodiscard]] std::uint64_t dropped() const noexcept { return dropped_; }

    static constexpr std::size_t kMaxBuffered = 1u << 16;

private:
    bool enabled_ = false;
    std::uint64_t tick_ = 0;
    std::uint64_t dropped_ = 0;
    std::vector<SimEvent> events_;
};

}  // namespace thalassa::sim
