add_library(thalassa_server STATIC
    src/headless_app.cpp
)
add_library(thalassa::server ALIAS thalassa_server)

target_include_directories(thalassa_server
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(thalassa_server
    PUBLIC
        thalassa::core
        thalassa::sim
        thalassa::net
)

target_compile_features(thalassa_server PUBLIC cxx_std_20)

thalassa_set_warnings(thalassa_server)
thalassa_apply_deterministic_fp(thalassa_server)
