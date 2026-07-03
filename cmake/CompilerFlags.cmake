# Determinism-critical compiler flags. Every target that contains
# simulation code must call maul2d_apply_flags(). The FMA canary in the
# test suite verifies at runtime what these flags promise at build time.

function(maul2d_apply_flags target)
    set_target_properties(${target} PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)

    if(MSVC)
        # /fp:precise alone permitted FMA contraction before VS2022 17.0
        # (and by default on arm64). Require a compiler that knows
        # /fp:contract so contraction can be switched off explicitly.
        if(MSVC_VERSION LESS 1930)
            message(FATAL_ERROR "MSVC >= VS2022 17.0 (1930) required: older versions cannot disable FP contraction")
        endif()
        target_compile_options(${target} PRIVATE /W4 /WX /fp:precise /fp:contract-)
    else()
        target_compile_options(${target} PRIVATE
            -ffp-contract=off
            -Wall -Wextra -Werror -Wshadow -Wdouble-promotion)
    endif()

    if(MAUL2D_SANITIZE)
        if(NOT MSVC)
            target_compile_options(${target} PRIVATE -fsanitize=address,undefined -fno-sanitize-recover=all)
            target_link_options(${target} PRIVATE -fsanitize=address,undefined)
        endif()
    endif()
endfunction()
