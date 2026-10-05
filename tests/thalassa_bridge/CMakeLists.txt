add_executable(thalassa_bridge_tests
    coordinate_conversion_tests.cpp
    entity_handle_tests.cpp
    command_builders_tests.cpp
)

target_link_libraries(thalassa_bridge_tests
    PRIVATE
        thalassa::core
        thalassa::sim
        thalassa::bridge
        Catch2::Catch2WithMain
)

thalassa_set_warnings(thalassa_bridge_tests)
thalassa_apply_deterministic_fp(thalassa_bridge_tests)

include(CTest)
add_test(NAME thalassa_bridge_tests COMMAND thalassa_bridge_tests)
