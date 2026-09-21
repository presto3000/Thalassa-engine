add_executable(thalassa_core_tests
    ecs_tests.cpp
    strictfp_tests.cpp
    random_and_timestep_tests.cpp
    vector_tests.cpp
)

target_link_libraries(thalassa_core_tests
    PRIVATE
        thalassa::core
        Catch2::Catch2WithMain
)

thalassa_set_warnings(thalassa_core_tests)
thalassa_apply_deterministic_fp(thalassa_core_tests)

# Note: we deliberately register the whole Catch2 binary as a single CTest
# test rather than using catch_discover_tests()'s per-case registration.
# Catch2's discovery integration bundles remaining cases into one ctest
# entry once the argument-length heuristic kicks in, which produced a
# broken filter expression in local testing. `ctest --preset ...` is meant
# as a quick pass/fail smoke check in this repo; for per-test triage, run
# the binary directly with `--list-tests` / a tag filter.
include(CTest)
add_test(NAME thalassa_core_tests COMMAND thalassa_core_tests)
