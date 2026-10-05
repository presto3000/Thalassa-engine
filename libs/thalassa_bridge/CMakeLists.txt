add_library(thalassa_bridge STATIC
    src/bridge.cpp
)
add_library(thalassa::bridge ALIAS thalassa_bridge)

target_include_directories(thalassa_bridge
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(thalassa_bridge
    PUBLIC
        thalassa::core
        thalassa::sim
)

target_compile_features(thalassa_bridge PUBLIC cxx_std_20)

thalassa_set_warnings(thalassa_bridge)
thalassa_apply_deterministic_fp(thalassa_bridge)

# thalassa_bridge must stay free of Unreal Engine headers/types — it's the part of the UE5
# integration this repo's own toolchain can actually build and test.
