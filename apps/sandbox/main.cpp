#include <bit>
#include <cstdint>
#include <cstdlib>

#include "thalassa/core/core.hpp"
#include "thalassa/server/headless_app.hpp"
#include "thalassa/sim/components.hpp"
#include "thalassa/sim/sim_world.hpp"

// Sandbox: a determinism smoke test that exercises thalassa_core's ECS, Rng, and math
// together end-to-end, run in fast-forward (non-real-time) mode so it's
// fast to run twice and diff. "5000+ units move deterministically, same inputs = same results on different
// machines" test will grow into — from just spawns a handful
// of entities with a toy component and checks that two independent
// SimWorld instances given the same seed and the same operations end up
// bit-identical.



namespace {

struct Position {
    thalassa::core::math::Vec3 value;
};

// Spawns N entities with a Position derived from a seeded Rng, and returns
// a simple checksum of the resulting state so two runs can be compared
// without printing every value.
std::uint64_t run_determinism_smoke_test(std::uint64_t seed, std::uint32_t entity_count) {
    thalassa::sim::SimWorld world(seed);

    for (std::uint32_t i = 0; i < entity_count; ++i) {
        const auto e = world.world().create_entity();
        const thalassa::core::Scalar x = world.rng().next_scalar01() * 100.0;
        const thalassa::core::Scalar y = world.rng().next_scalar01() * 100.0;
        const thalassa::core::Scalar z = 0.0;
        world.world().emplace<Position>(e, Position{{x, y, z}});
    }

    for (thalassa::core::SimTick t{0}; thalassa::core::to_u64(t) < 10ULL; t = t + 1) {
        world.tick(t);
    }

    std::uint64_t checksum = 0;
    world.world().each<Position>([&checksum](thalassa::core::ecs::Entity e, const Position& p) {
        // Simple order-sensitive FNV-1a-style fold over entity index and
        // position bits, to prove both *values* and *iteration order* are
        // reproduced identically across runs.
        auto mix = [&checksum](std::uint64_t v) {
            checksum ^= v;
            checksum *= 1099511628211ULL;
        };
        mix(e.index());
        mix(std::bit_cast<std::uint64_t>(p.value.x));
        mix(std::bit_cast<std::uint64_t>(p.value.y));
    });
    return checksum;
}

}  // namespace
using thalassa::sim::components::Destination;
using thalassa::sim::components::Position;

namespace {

    struct RunResult {
        std::uint64_t checksum = 0;
        std::size_t still_traveling = 0;
        std::size_t spatial_hits_near_origin = 0;
    };

    RunResult run_movement_smoke_test(std::uint64_t seed, std::uint32_t entity_count, int ticks) {
        thalassa::sim::SimWorld world(seed);

        for (std::uint32_t i = 0; i < entity_count; ++i) {
            const auto e = world.world().create_entity();
            const thalassa::core::Scalar sx = world.rng().next_scalar01() * 200.0 - 100.0;
            const thalassa::core::Scalar sy = world.rng().next_scalar01() * 200.0 - 100.0;
            const thalassa::core::Scalar tx = world.rng().next_scalar01() * 200.0 - 100.0;
            const thalassa::core::Scalar ty = world.rng().next_scalar01() * 200.0 - 100.0;
            const thalassa::core::Scalar speed = 10.0 + world.rng().next_scalar01() * 20.0;

            world.world().emplace<Position>(e, Position{ {sx, sy, 0.0} });
            world.world().emplace<Destination>(e, Destination{ {tx, ty, 0.0}, speed, 0.1 });
        }

        for (int t = 0; t < ticks; ++t) {
            world.tick(thalassa::core::SimTick{ static_cast<std::uint64_t>(t) });
        }

        RunResult result;
        world.world().each<Position>([&result](thalassa::core::ecs::Entity e, const Position& p) {
            auto mix = [&result](std::uint64_t v) {
                result.checksum ^= v;
                result.checksum *= 1099511628211ULL;
                };
            mix(e.index());
            mix(std::bit_cast<std::uint64_t>(p.value.x));
            mix(std::bit_cast<std::uint64_t>(p.value.y));
            });
        result.still_traveling = world.world().component_count<Destination>();

        std::vector<thalassa::core::ecs::Entity> nearby;
        world.spatial_index().query_radius({ 0.0, 0.0, 0.0 }, 20.0, nearby);
        result.spatial_hits_near_origin = nearby.size();

        return result;
    }
}

int main() {
    thalassa::core::log::set_min_level(thalassa::core::log::Level::Info);
    THALASSA_LOG_INFO("Thalassa sandbox: movement + spatial-index determinism smoke test");

    constexpr std::uint64_t kSeed = 42;
    constexpr std::uint32_t kEntityCount = 5000;
    constexpr int kTicks = 400;

    const RunResult a = run_movement_smoke_test(kSeed, kEntityCount, kTicks);
    const RunResult b = run_movement_smoke_test(kSeed, kEntityCount, kTicks);

    THALASSA_LOG_INFO("Run A checksum: 0x{:016x}, still traveling: {}, near origin (r=20): {}",
        a.checksum, a.still_traveling, a.spatial_hits_near_origin);
    THALASSA_LOG_INFO("Run B checksum: 0x{:016x}, still traveling: {}, near origin (r=20): {}",
        b.checksum, b.still_traveling, b.spatial_hits_near_origin);

    if (a.checksum != b.checksum) {
        THALASSA_LOG_ERROR("DETERMINISM VIOLATION: two identical runs produced different results!");
        return EXIT_FAILURE;
    }

    THALASSA_LOG_INFO("Deterministic: {} entities over {} ticks, identical checksum across two independent runs.",
        kEntityCount, kTicks);

    // Also demonstrate the fixed-timestep-driven HeadlessApp shape, in
    // fast-forward mode, for a few ticks.
    thalassa::server::HeadlessAppConfig config;
    config.max_ticks = 5;
    config.real_time = false;
    thalassa::server::HeadlessApp app(config);
    app.run();
    THALASSA_LOG_INFO("HeadlessApp fast-forward run complete: {} ticks simulated.",
        app.sim_world().ticks_simulated());

    return EXIT_SUCCESS;
}
