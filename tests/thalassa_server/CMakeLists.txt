add_executable(thalassa_server_tests
    match_host_tests.cpp
)

target_link_libraries(thalassa_server_tests
    PRIVATE
        thalassa::core
        thalassa::sim
        thalassa::net
        thalassa::server
        Catch2::Catch2WithMain
)

thalassa_set_warnings(thalassa_server_tests)
thalassa_apply_deterministic_fp(thalassa_server_tests)

include(CTest)
add_test(NAME thalassa_server_tests COMMAND thalassa_server_tests)
