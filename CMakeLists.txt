cmake_minimum_required(VERSION 3.20)

project(Thalassa
    VERSION 0.1.0
    DESCRIPTION "Deterministic high-performance multiplayer simulation engine"
    LANGUAGES CXX
)

# ---------------------------------------------------------------------------
# Global project options
# ---------------------------------------------------------------------------
option(THALASSA_BUILD_RENDER   "Build the optional thalassa_render module (Vulkan/DX12)" OFF)
option(THALASSA_BUILD_SERVER   "Build the thalassa_server headless app"                   ON)
option(THALASSA_BUILD_SANDBOX  "Build the local sandbox/debug app"                        ON)
option(THALASSA_BUILD_TESTS    "Build unit tests"                                         ON)
option(THALASSA_WARNINGS_AS_ERRORS "Treat compiler warnings as errors"                    ON)
option(THALASSA_USE_FIXED_POINT "Use fixed-point math for deterministic sim values"       OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Position-independent code everywhere: thalassa_core/sim/net must be
# link-compatible with UE5's shared-library modules.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Build type" FORCE)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(CompilerWarnings)
include(DeterministicMath)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
include(FetchContent)

# fmt: header-quality, fast, deterministic formatting. Used only for logging;
# never used in a way that affects simulation state.
FetchContent_Declare(
    fmt
    GIT_REPOSITORY https://github.com/fmtlib/fmt.git
    GIT_TAG        11.0.2
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(fmt)

if(THALASSA_BUILD_TESTS)
    FetchContent_Declare(
        catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.6.0
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(catch2)
    list(APPEND CMAKE_MODULE_PATH "${catch2_SOURCE_DIR}/extras")
endif()

# ---------------------------------------------------------------------------
# Core libraries (always built — these must remain UE5-safe: no platform
# headers, no rendering types, no exceptions across public API boundaries)
# ---------------------------------------------------------------------------
add_subdirectory(libs/thalassa_core)
add_subdirectory(libs/thalassa_sim)
add_subdirectory(libs/thalassa_net)

if(THALASSA_BUILD_SERVER)
    add_subdirectory(libs/thalassa_server)
endif()

# if(THALASSA_BUILD_RENDER)
#     add_subdirectory(libs/thalassa_render)
# endif()

if(THALASSA_BUILD_SERVER)
    add_subdirectory(apps/headless_server)
endif()

if(THALASSA_BUILD_SANDBOX)
    add_subdirectory(apps/sandbox)
endif()

if(THALASSA_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests/thalassa_core)
    add_subdirectory(tests/thalassa_sim)
endif()

message(STATUS "==================================================")
message(STATUS "Thalassa configuration")
message(STATUS "  Build type          : ${CMAKE_BUILD_TYPE}")
message(STATUS "  Fixed-point sim math : ${THALASSA_USE_FIXED_POINT}")
message(STATUS "  Render module        : ${THALASSA_BUILD_RENDER}")
message(STATUS "  Server app           : ${THALASSA_BUILD_SERVER}")
message(STATUS "  Sandbox app          : ${THALASSA_BUILD_SANDBOX}")
message(STATUS "  Tests                : ${THALASSA_BUILD_TESTS}")
message(STATUS "==================================================")
