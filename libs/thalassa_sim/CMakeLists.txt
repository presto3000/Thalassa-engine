add_library(thalassa_sim STATIC
    src/sim_world.cpp
    src/systems/movement.cpp
    src/replay.cpp
)
add_library(thalassa::sim ALIAS thalassa_sim)

target_include_directories(thalassa_sim
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(thalassa_sim
    PUBLIC
        thalassa::core
)

target_compile_features(thalassa_sim PUBLIC cxx_std_20)

thalassa_set_warnings(thalassa_sim)
thalassa_apply_deterministic_fp(thalassa_sim)
