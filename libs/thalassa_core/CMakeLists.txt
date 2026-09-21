add_library(thalassa_core STATIC
    src/log.cpp
    src/math/strictfp.cpp
)
add_library(thalassa::core ALIAS thalassa_core)

target_include_directories(thalassa_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(thalassa_core
    PUBLIC
        fmt::fmt
)

target_compile_features(thalassa_core PUBLIC cxx_std_20)

thalassa_set_warnings(thalassa_core)
thalassa_apply_deterministic_fp(thalassa_core)

# thalassa_core's PUBLIC headers must never depend on rendering or
# platform-SDK headers (Unreal reuse requirement). This is enforced by
# convention + code review today; a header-hygiene CI check (e.g. grepping
# for windows.h/vulkan.h/Unreal includes under include/) is a good addition
# once CI exists.
