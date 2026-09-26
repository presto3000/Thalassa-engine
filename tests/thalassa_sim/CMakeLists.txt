add_executable(thalassa_sim_tests
    movement_tests.cpp
    spatial_index_tests.cpp
    replay_tests.cpp
    combat_tests.cpp
    command_tests.cpp
)

target_link_libraries(thalassa_sim_tests
    PRIVATE
        thalassa::core
        thalassa::sim
        Catch2::Catch2WithMain
)

thalassa_set_warnings(thalassa_sim_tests)
thalassa_apply_deterministic_fp(thalassa_sim_tests)

include(CTest)
add_test(NAME thalassa_sim_tests COMMAND thalassa_sim_tests)
