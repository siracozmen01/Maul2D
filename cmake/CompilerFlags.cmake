# Determinism-critical compiler flags. Every target that contains
# simulation code must call maul2d_apply_flags(). The FMA canary in the
# test suite verifies at runtime what these flags promise at build time.

# SIMD backend policy (slice 60, user decision 2026-07-06): x64
# baseline is AVX2+FMA (Haswell 2013+); arm64 uses NEON, which is
# architectural. MAUL2D_SIMD=scalar forces the fmaf fallback, and CI
# runs one scalar cell whose hashes must match the vector cells bit
# for bit: the backends are interchangeable or the build is red.
set(MAUL2D_SIMD "auto" CACHE STRING "SIMD backend: auto or scalar")

function(maul2d_apply_flags target)
    # A host injecting fast-math through global flags would silently void
    # the determinism contract; refuse to configure at all.
    string(FIND "${CMAKE_C_FLAGS}" "fast-math" _m2_fastmath)
    string(FIND "${CMAKE_C_FLAGS}" "/fp:fast" _m2_fpfast)
    if(NOT _m2_fastmath EQUAL -1 OR NOT _m2_fpfast EQUAL -1)
        message(FATAL_ERROR "fast-math in CMAKE_C_FLAGS is incompatible with maul2d's determinism contract")
    endif()

    set_target_properties(${target} PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)

    if(MSVC)
        # /fp:precise alone permitted FMA contraction before VS2022 17.0
        # (and by default on arm64). Require a compiler that knows
        # /fp:contract so contraction can be switched off explicitly.
        if(MSVC_VERSION LESS 1930)
            message(FATAL_ERROR "MSVC >= VS2022 17.0 (1930) required: older versions cannot disable FP contraction")
        endif()
        target_compile_options(${target} PRIVATE /W4 /WX /fp:precise /fp:contract-)
        if(CMAKE_SYSTEM_PROCESSOR MATCHES "AMD64|x86_64" AND NOT MAUL2D_SIMD STREQUAL "scalar")
            target_compile_options(${target} PRIVATE /arch:AVX2)
        endif()
    else()
        target_compile_options(${target} PRIVATE
            -ffp-contract=off -fno-trapping-math -fno-fast-math -fno-unsafe-math-optimizations
            -Wall -Wextra -Werror -Wshadow -Wdouble-promotion)
        if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64" AND NOT MAUL2D_SIMD STREQUAL "scalar")
            target_compile_options(${target} PRIVATE -mavx2 -mfma)
        endif()
    endif()

    if(MAUL2D_SIMD STREQUAL "scalar")
        target_compile_definitions(${target} PRIVATE MAUL2D_SIMD_FORCE_SCALAR=1)
    endif()

    if(MAUL2D_SANITIZE)
        if(NOT MSVC)
            target_compile_options(${target} PRIVATE -fsanitize=address,undefined -fno-sanitize-recover=all)
            target_link_options(${target} PRIVATE -fsanitize=address,undefined)
        endif()
    endif()
    if(MAUL2D_TSAN)
        if(NOT MSVC)
            target_compile_options(${target} PRIVATE -fsanitize=thread)
            target_link_options(${target} PRIVATE -fsanitize=thread)
        endif()
    endif()
    if(MAUL2D_VALIDATE)
        target_compile_definitions(${target} PRIVATE MAUL2D_VALIDATE=1)
    endif()
endfunction()
