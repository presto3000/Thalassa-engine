add_executable(thalassa_net_tests
    snapshot_delta_tests.cpp
    predicted_client_tests.cpp
    loopback_transport_tests.cpp
    reliable_channel_tests.cpp
    protocol_tests.cpp
)

if(NOT WIN32)
    target_sources(thalassa_net_tests PRIVATE udp_transport_tests.cpp)
endif()

target_link_libraries(thalassa_net_tests
    PRIVATE
        thalassa::core
        thalassa::sim
        thalassa::net
        Catch2::Catch2WithMain
)

thalassa_set_warnings(thalassa_net_tests)
thalassa_apply_deterministic_fp(thalassa_net_tests)

include(CTest)
add_test(NAME thalassa_net_tests COMMAND thalassa_net_tests)
