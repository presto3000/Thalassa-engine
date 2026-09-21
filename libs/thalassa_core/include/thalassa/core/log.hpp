#pragma once

#include <string_view>

#include <fmt/format.h>

#include "thalassa/core/config.hpp"

// Minimal, dependency-light logging. Deliberately NOT used by anything on
// the deterministic simulation path in a way that could affect state (i.e.
// never gate simulation branches on whether logging is enabled) — logging
// is diagnostic-only. Sinks are pluggable so the headless server can log to
// stdout/file while a future editor/tool can add its own sink without
// touching thalassa_core.

namespace thalassa::core::log {

enum class Level : int {
    Trace = 0,
    Debug = 1,
    Info = 2,
    Warn = 3,
    Error = 4,
};

[[nodiscard]] constexpr std::string_view to_string(Level level) noexcept {
    switch (level) {
        case Level::Trace: return "TRACE";
        case Level::Debug: return "DEBUG";
        case Level::Info:  return "INFO";
        case Level::Warn:  return "WARN";
        case Level::Error: return "ERROR";
    }
    return "?????";
}

// A sink receives already-formatted lines. Implementations must be
// thread-safe if logging happens from multiple threads; the
// default sink below uses a mutex internally.
class Sink {
public:
    virtual ~Sink() = default;
    virtual void write(Level level, std::string_view message) = 0;
};

// Installs a sink globally, replacing whatever was previously installed.
// Not called from any header-only inline function so this stays an
// ODR-safe, single definition in log.cpp. Passing nullptr restores the
// default stdout sink.
void set_sink(Sink* sink);

// Sets the minimum level that will reach the sink. Default: Info.
void set_min_level(Level level);
[[nodiscard]] Level min_level();

namespace detail {
void dispatch(Level level, std::string_view message);
}  // namespace detail

template <typename... Args>
void log(Level level, fmt::format_string<Args...> fmt_str, Args&&... args) {
    if (level < min_level()) {
        return;
    }
    detail::dispatch(level, fmt::format(fmt_str, std::forward<Args>(args)...));
}

}  // namespace thalassa::core::log

#define THALASSA_LOG_TRACE(...) ::thalassa::core::log::log(::thalassa::core::log::Level::Trace, __VA_ARGS__)
#define THALASSA_LOG_DEBUG(...) ::thalassa::core::log::log(::thalassa::core::log::Level::Debug, __VA_ARGS__)
#define THALASSA_LOG_INFO(...)  ::thalassa::core::log::log(::thalassa::core::log::Level::Info,  __VA_ARGS__)
#define THALASSA_LOG_WARN(...)  ::thalassa::core::log::log(::thalassa::core::log::Level::Warn,  __VA_ARGS__)
#define THALASSA_LOG_ERROR(...) ::thalassa::core::log::log(::thalassa::core::log::Level::Error, __VA_ARGS__)
