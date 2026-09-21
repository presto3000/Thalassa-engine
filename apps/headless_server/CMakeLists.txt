add_executable(thalassa_headless_server
    main.cpp
)

target_link_libraries(thalassa_headless_server
    PRIVATE
        thalassa::core
        thalassa::sim
        thalassa::net
        thalassa::server
)

thalassa_set_warnings(thalassa_headless_server)
thalassa_apply_deterministic_fp(thalassa_headless_server)
