cmake_minimum_required(VERSION 3.22)
project(maul2d LANGUAGES C)

option(MAUL2D_SANITIZE "Build with ASAN/UBSAN" OFF)

include(cmake/CompilerFlags.cmake)

add_library(maul2d STATIC
    src/core.c
    src/math_functions.c)
target_include_directories(maul2d PUBLIC include)
maul2d_apply_flags(maul2d)

enable_testing()

add_executable(test_core test/test_core.c)
target_link_libraries(test_core PRIVATE maul2d)
maul2d_apply_flags(test_core)
if(NOT MSVC)
    # Tests compare against libm references.
    target_link_libraries(test_core PRIVATE m)
endif()
add_test(NAME core COMMAND test_core)
