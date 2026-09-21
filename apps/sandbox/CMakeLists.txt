add_executable(thalassa_sandbox
    main.cpp
)

target_link_libraries(thalassa_sandbox
    PRIVATE
        thalassa::core
        thalassa::sim
        thalassa::net
        thalassa::server
)

thalassa_set_warnings(thalassa_sandbox)
thalassa_apply_deterministic_fp(thalassa_sandbox)
