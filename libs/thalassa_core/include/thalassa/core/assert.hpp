#pragma once

#include <cstdlib>

#include "thalassa/core/log.hpp"

// THALASSA_ASSERT: always compiled in (even in release), because a
// desynced deterministic simulation is worse than a slightly slower
// release build — silently continuing after an invariant violation on the
// sim path is exactly the kind of bug this project cannot afford. Use
// THALASSA_ASSERT_SLOW for checks that are too expensive to keep in
// release (e.g. O(n) validation inside a hot per-entity loop); those
// compile out unless THALASSA_SLOW_ASSERTS is defined.
//
// Deliberately does not throw (coding standard: no exceptions in hot
// paths) — it logs and aborts, giving a debuggable core dump instead of
// silently corrupting simulation state that would then desync clients from
// the server.

namespace thalassa::core::detail {

[[noreturn]] inline void assert_fail(const char* expr, const char* file, int line, const char* message) {
    THALASSA_LOG_ERROR("ASSERTION FAILED: {} at {}:{}{}{}",
                        expr, file, line,
                        message != nullptr ? " -- " : "",
                        message != nullptr ? message : "");
    std::abort();
}

}  // namespace thalassa::core::detail

#define THALASSA_ASSERT(expr)                                                          \
    do {                                                                               \
        if (!(expr)) {                                                                 \
            ::thalassa::core::detail::assert_fail(#expr, __FILE__, __LINE__, nullptr); \
        }                                                                              \
    } while (false)

#define THALASSA_ASSERT_MSG(expr, msg)                                              \
    do {                                                                            \
        if (!(expr)) {                                                              \
            ::thalassa::core::detail::assert_fail(#expr, __FILE__, __LINE__, msg);  \
        }                                                                           \
    } while (false)

#if defined(THALASSA_SLOW_ASSERTS)
    #define THALASSA_ASSERT_SLOW(expr) THALASSA_ASSERT(expr)
#else
    #define THALASSA_ASSERT_SLOW(expr) \
        do {                           \
        } while (false)
#endif
