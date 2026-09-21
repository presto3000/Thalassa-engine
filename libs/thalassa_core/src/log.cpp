#include "thalassa/core/log.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <mutex>

#include <fmt/chrono.h>

namespace thalassa::core::log {

namespace {

class StdoutSink final : public Sink {
public:
    void write(Level level, std::string_view message) override {
        std::scoped_lock lock(mutex_);
        const auto now = std::chrono::system_clock::now();
        std::FILE* stream = (level >= Level::Warn) ? stderr : stdout;
        // fmt's chrono formatter already prints sub-second precision for a
        // system_clock::time_point via %S (it infers precision from the
        // duration's period), so no separate millisecond field is needed.
        fmt::print(stream, "[{:%H:%M:%S}][{}] {}\n", now, to_string(level), message);
        std::fflush(stream);
    }

private:
    std::mutex mutex_;
};

StdoutSink g_default_sink;
std::atomic<Sink*> g_sink{&g_default_sink};
std::atomic<Level> g_min_level{Level::Info};

}  // namespace

void set_sink(Sink* sink) {
    g_sink.store(sink != nullptr ? sink : &g_default_sink, std::memory_order_relaxed);
}

void set_min_level(Level level) {
    g_min_level.store(level, std::memory_order_relaxed);
}

Level min_level() {
    return g_min_level.load(std::memory_order_relaxed);
}

namespace detail {

void dispatch(Level level, std::string_view message) {
    g_sink.load(std::memory_order_relaxed)->write(level, message);
}

}  // namespace detail

}  // namespace thalassa::core::log
