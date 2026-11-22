#include "thalassa/sim/group_move.hpp"

#include <algorithm>
#include <cmath>

#include "thalassa/core/ecs/world.hpp"
#include "thalassa/core/math/strictfp.hpp"
#include "thalassa/sim/components.hpp"

namespace thalassa::sim {

using core::Scalar;
using core::ecs::Entity;
using core::math::Vec3;

std::vector<Vec3> spread_slots(const Vec3& center, std::size_t count, Scalar spacing) {
    std::vector<Vec3> out;
    out.reserve(count);
    if (count == 0) return out;
    out.push_back(center);
    // Hex rings: ring k has 6k slots; its corner points are k steps along the six axes, the rest are
    // interpolated along the sides. Integer-only indexing, so the result is identical everywhere.
    static const Scalar kDirX[6] = {1.0, 0.5, -0.5, -1.0, -0.5, 0.5};
    static const Scalar kDirY[6] = {0.0, 0.8660254037844386, 0.8660254037844386, 0.0, -0.8660254037844386,
                                    -0.8660254037844386};
    for (int ring = 1; out.size() < count; ++ring) {
        for (int side = 0; side < 6 && out.size() < count; ++side) {
            const int next = (side + 1) % 6;
            for (int step = 0; step < ring && out.size() < count; ++step) {
                const Scalar t = static_cast<Scalar>(step) / static_cast<Scalar>(ring);
                const Scalar x = (kDirX[side] * (1.0 - t) + kDirX[next] * t) * static_cast<Scalar>(ring) * spacing;
                const Scalar y = (kDirY[side] * (1.0 - t) + kDirY[next] * t) * static_cast<Scalar>(ring) * spacing;
                out.push_back(Vec3{center.x + x, center.y + y, center.z});
            }
        }
    }
    return out;
}

std::vector<Command> plan_group_move(const core::ecs::World& world, const std::vector<Entity>& units,
                                     const Vec3& center, Scalar speed, Scalar spacing) {
    struct Item {
        Entity entity;
        Scalar dist_sq;
    };
    std::vector<Item> items;
    items.reserve(units.size());
    for (const Entity e : units) {
        if (!world.is_alive(e) || !world.has<components::Position>(e)) continue;
        const Vec3 d = world.get<components::Position>(e).value - center;
        items.push_back(Item{e, d.x * d.x + d.y * d.y});
    }
    std::sort(items.begin(), items.end(), [](const Item& a, const Item& b) {
        if (a.dist_sq != b.dist_sq) return a.dist_sq < b.dist_sq;
        return a.entity.index() < b.entity.index();
    });
    const std::vector<Vec3> slots = spread_slots(center, items.size(), spacing);
    std::vector<Command> commands;
    commands.reserve(items.size());
    for (std::size_t i = 0; i < items.size(); ++i) commands.push_back(Command::move_to(items[i].entity, slots[i], speed));
    return commands;
}

}  // namespace thalassa::sim
