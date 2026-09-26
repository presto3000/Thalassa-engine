add_library(thalassa_net STATIC
    src/snapshot_delta.cpp
    src/predicted_client.cpp
)
add_library(thalassa::net ALIAS thalassa_net)

target_include_directories(thalassa_net
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(thalassa_net
    PUBLIC
        thalassa::core
        thalassa::sim
)

target_compile_features(thalassa_net PUBLIC cxx_std_20)

thalassa_set_warnings(thalassa_net)
thalassa_apply_deterministic_fp(thalassa_net)
