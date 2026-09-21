# CompilerWarnings.cmake
#
# thalassa_set_warnings(<target>) applies a strict, consistent warning set
# across every module. Kept in one place so simulation-affecting warnings
# (e.g. narrowing conversions, sign compares) are never silently disabled
# in one library but not another.

function(thalassa_set_warnings target)
    set(MSVC_WARNINGS
        /W4
        /permissive-
        /w14242 /w14254 /w14263 /w14265 /w14287
        /we4289
        /w14296 /w14311 /w14545 /w14546 /w14547
        /w14549 /w14555 /w14619 /w14640 /w14826
        /w14905 /w14906 /w14928
    )

    set(CLANG_GCC_WARNINGS
        -Wall
        -Wextra
        -Wshadow
        -Wnon-virtual-dtor
        -Wold-style-cast
        -Wcast-align
        -Wunused
        -Woverloaded-virtual
        -Wpedantic
        -Wconversion
        -Wsign-conversion
        -Wnull-dereference
        -Wdouble-promotion
        -Wformat=2
        -Wimplicit-fallthrough
    )

    if(THALASSA_WARNINGS_AS_ERRORS)
        list(APPEND CLANG_GCC_WARNINGS -Werror)
        list(APPEND MSVC_WARNINGS /WX)
    endif()

    if(MSVC)
        target_compile_options(${target} PRIVATE ${MSVC_WARNINGS})
    else()
        target_compile_options(${target} PRIVATE ${CLANG_GCC_WARNINGS})
    endif()
endfunction()
