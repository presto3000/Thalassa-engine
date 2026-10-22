#include <csignal>
#include <cstdlib>
#include <cstring>
#include <string>

#include "thalassa/core/log.hpp"
#include "thalassa/server/dedicated_server.hpp"

// Thalassa dedicated server: hosts one match over UDP until Ctrl-C / SIGTERM.
//
//   thalassa_headless_server [--port N] [--bind ADDR] [--seed N] [--players N]
//                            [--replay FILE] [--max-clients N]
//
// Clients connect to <host>:<port> and join the printed match id.

namespace {
thalassa::server::DedicatedServer* g_server = nullptr;
void handle_signal(int) {
    if (g_server != nullptr) g_server->stop();
}

void usage() {
    THALASSA_LOG_INFO(
        "usage: thalassa_headless_server [--port N] [--bind ADDR] [--seed N] [--players N] "
        "[--replay FILE] [--max-clients N]");
}
}  // namespace

int main(int argc, char** argv) {
    thalassa::core::log::set_min_level(thalassa::core::log::Level::Info);
    thalassa::server::DedicatedServerConfig config;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const bool has_value = i + 1 < argc;
        try {
            if (arg == "--port" && has_value) config.port = static_cast<std::uint16_t>(std::stoul(argv[++i]));
            else if (arg == "--bind" && has_value) config.bind_address = argv[++i];
            else if (arg == "--seed" && has_value) config.seed = std::stoull(argv[++i]);
            else if (arg == "--players" && has_value) config.players = static_cast<std::uint32_t>(std::stoul(argv[++i]));
            else if (arg == "--replay" && has_value) config.replay_path = argv[++i];
            else if (arg == "--max-clients" && has_value) config.server.max_clients = std::stoul(argv[++i]);
            else { usage(); return arg == "--help" ? EXIT_SUCCESS : EXIT_FAILURE; }
        } catch (const std::exception&) {
            usage();
            return EXIT_FAILURE;
        }
    }

    std::string error;
    auto server = thalassa::server::DedicatedServer::create(config, &error);
    if (server == nullptr) {
        THALASSA_LOG_ERROR("cannot start server: {}", error);
        return EXIT_FAILURE;
    }
    g_server = server.get();
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    THALASSA_LOG_INFO("Thalassa dedicated server listening on {}:{}  match id {}  seed {}  arena for {} players",
                      config.bind_address, server->port(), server->match_id().value(), config.seed, config.players);
    server->run();

    const auto& stats = server->game().stats();
    THALASSA_LOG_INFO("shutting down: tick {}, joined {}, dropped {}, commands ok/rejected {}/{}",
                      thalassa::core::to_u64(server->game().host().match_tick(server->match_id())),
                      stats.clients_joined, stats.clients_dropped, stats.commands_accepted, stats.commands_rejected);
    if (!config.replay_path.empty()) {
        if (server->save_replay()) THALASSA_LOG_INFO("replay written to {}", config.replay_path.string());
        else THALASSA_LOG_ERROR("failed to write replay to {}", config.replay_path.string());
    }
    g_server = nullptr;
    return EXIT_SUCCESS;
}
