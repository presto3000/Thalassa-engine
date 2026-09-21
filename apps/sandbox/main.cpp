#include <bit>
#include <cstdint>
#include <cstdlib>

#include "thalassa/core/core.hpp"
#include "thalassa/server/headless_app.hpp"

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

int main() {
    thalassa::core::log::set_min_level(thalassa::core::log::Level::Info);
    THALASSA_LOG_INFO("Thalassa sandbox: determinism smoke test (Milestone 0)");

    constexpr std::uint64_t kSeed = 42;
    constexpr std::uint32_t kEntityCount = 5000;

    const std::uint64_t checksum_a = run_determinism_smoke_test(kSeed, kEntityCount);
    const std::uint64_t checksum_b = run_determinism_smoke_test(kSeed, kEntityCount);

    THALASSA_LOG_INFO("Run A checksum: 0x{:016x}", checksum_a);
    THALASSA_LOG_INFO("Run B checksum: 0x{:016x}", checksum_b);

    if (checksum_a != checksum_b) {
        THALASSA_LOG_ERROR("DETERMINISM VIOLATION: two identical runs produced different results!");
        return EXIT_FAILURE;
    }

    THALASSA_LOG_INFO("Deterministic: {} entities, identical checksum across two independent runs.",
                       kEntityCount);

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
