#include <csignal>
#include <cstdlib>

#include "thalassa/core/log.hpp"
#include "thalassa/server/headless_app.hpp"

// a headless application that runs a fixed
// timestep loop with an empty ECS world, cleanly stoppable via Ctrl-C.

namespace {
thalassa::server::HeadlessApp* g_app_for_signal_handler = nullptr;

void handle_sigint(int) {
    if (g_app_for_signal_handler != nullptr) {
        g_app_for_signal_handler->stop();
    }
}
}  // namespace

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    thalassa::core::log::set_min_level(thalassa::core::log::Level::Info);
    THALASSA_LOG_INFO("Thalassa headless server starting");

    thalassa::server::HeadlessAppConfig config;
    config.ticks_per_second = 60;
    config.rng_seed = 1;
    config.max_ticks = 0;      // run until Ctrl-C
    config.real_time = true;

    thalassa::server::HeadlessApp app(config);
    g_app_for_signal_handler = &app;
    std::signal(SIGINT, handle_sigint);
    std::signal(SIGTERM, handle_sigint);

    THALASSA_LOG_INFO("Fixed timestep: {} Hz, empty ECS world, entities alive: {}",
                       config.ticks_per_second, app.sim_world().world().alive_count());

    app.run();

    THALASSA_LOG_INFO("Shutting down after {} simulated ticks.", app.sim_world().ticks_simulated());
    return EXIT_SUCCESS;
}
