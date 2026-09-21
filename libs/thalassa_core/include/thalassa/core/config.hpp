#pragma once

// config.hpp
//
// Compiler/platform feature detection macros ONLY. This header must never
// pull in a platform SDK header (windows.h, etc). It exists so the rest of
// thalassa_core can express intent (THALASSA_LIKELY, THALASSA_FORCE_INLINE)
// without every file re-deriving compiler quirks, keeping the public
// headers safe to include from an Unreal Engine 5 module.

#if defined(_MSC_VER)
    #define THALASSA_COMPILER_MSVC 1
#elif defined(__clang__)
    #define THALASSA_COMPILER_CLANG 1
#elif defined(__GNUC__)
    #define THALASSA_COMPILER_GCC 1
#endif

#if defined(_WIN32)
    #define THALASSA_PLATFORM_WINDOWS 1
#elif defined(__linux__)
    #define THALASSA_PLATFORM_LINUX 1
#elif defined(__APPLE__)
    #define THALASSA_PLATFORM_MAC 1
#endif

#if defined(THALASSA_COMPILER_MSVC)
    #define THALASSA_FORCE_INLINE __forceinline
#else
    #define THALASSA_FORCE_INLINE inline __attribute__((always_inline))
#endif

#if defined(THALASSA_COMPILER_MSVC)
    #define THALASSA_LIKELY(x)   (x)
    #define THALASSA_UNLIKELY(x) (x)
#else
    #define THALASSA_LIKELY(x)   __builtin_expect(!!(x), 1)
    #define THALASSA_UNLIKELY(x) __builtin_expect(!!(x), 0)
#endif

// Shared/static library export markers. THALASSA_CORE_SHARED is defined by
// CMake only when building thalassa_core as a shared library (e.g. for
// hot-reloadable tooling); default builds are static and this expands to
// nothing, which is what the UE5 plugin (statically linking) wants.
#if defined(THALASSA_CORE_SHARED)
    #if defined(THALASSA_PLATFORM_WINDOWS)
        #if defined(THALASSA_CORE_BUILDING)
            #define THALASSA_CORE_API __declspec(dllexport)
        #else
            #define THALASSA_CORE_API __declspec(dllimport)
        #endif
    #else
        #define THALASSA_CORE_API __attribute__((visibility("default")))
    #endif
#else
    #define THALASSA_CORE_API
#endif
