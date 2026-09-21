# DeterministicMath.cmake
#
# Pins floating-point codegen so that the *same binary built for the same
# target architecture* produces bit-identical simulation results across
# runs and across machines with the same ISA. This is the compiler-level
# half of the floating-point determinism policy described in
# docs/floating_point_policy.md. It does NOT guarantee bit-identical results
# across different CPU vendors/microarchitectures with differing FPU
# behavior (e.g. x86 vs ARM, or machines with different SSE/AVX levels);
# that is why thalassa_core additionally ships a soft-float scalar math
# library (thalassa::core::math::strictfp) for the operations that appear
# directly in the deterministic simulation step.
#
# thalassa_apply_deterministic_fp(<target>) applies these flags.

function(thalassa_apply_deterministic_fp target)
    if(MSVC)
        # /fp:strict disables reassociation and contraction (no FMA fusing
        # unless explicitly requested), matching IEEE-754 semantics exactly.
        target_compile_options(${target} PRIVATE /fp:strict)
    else()
        target_compile_options(${target} PRIVATE
            -fno-fast-math
            -ffp-contract=off      # no implicit fused multiply-add
            -frounding-math
            -fno-unsafe-math-optimizations
        )
        # Pin SSE2 as the floating-point unit on x86 so codegen doesn't vary
        # between machines that do/don't have AVX. On Linux CI this must be
        # matched by the deployment target; document any change here.
        include(CheckCXXCompilerFlag)
        check_cxx_compiler_flag("-mfpmath=sse" THALASSA_HAS_MFPMATH_SSE)
        if(THALASSA_HAS_MFPMATH_SSE)
            target_compile_options(${target} PRIVATE -mfpmath=sse -msse2)
        endif()
    endif()
endfunction()
