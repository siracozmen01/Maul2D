cmake_minimum_required(VERSION 3.22)
project(maul2d LANGUAGES C)

option(MAUL2D_SANITIZE "Build with ASAN/UBSAN" OFF)

include(cmake/CompilerFlags.cmake)

add_library(maul2d STATIC
    src/core.c
    src/dynamic_tree.c
    src/manifold.c
    src/math_functions.c
    src/shape.c
    src/solver.c
    src/world.c)
target_include_directories(maul2d PUBLIC include)
maul2d_apply_flags(maul2d)
if(NOT MSVC)
    # The library's own math (sqrtf/floorf) needs libm; consumers inherit it.
    target_link_libraries(maul2d PUBLIC m)
endif()

enable_testing()

add_executable(test_core test/test_core.c)
target_link_libraries(test_core PRIVATE maul2d)
maul2d_apply_flags(test_core)
add_test(NAME core COMMAND test_core)

add_executable(test_world test/test_world.c)
target_link_libraries(test_world PRIVATE maul2d)
maul2d_apply_flags(test_world)
add_test(NAME world COMMAND test_world)

add_executable(test_broadphase test/test_broadphase.c)
target_link_libraries(test_broadphase PRIVATE maul2d)
target_include_directories(test_broadphase PRIVATE src)
maul2d_apply_flags(test_broadphase)
add_test(NAME broadphase COMMAND test_broadphase)

add_executable(test_shapes test/test_shapes.c)
target_link_libraries(test_shapes PRIVATE maul2d)
target_include_directories(test_shapes PRIVATE src)
maul2d_apply_flags(test_shapes)
add_test(NAME shapes COMMAND test_shapes)

add_executable(test_contacts test/test_contacts.c)
target_link_libraries(test_contacts PRIVATE maul2d)
target_include_directories(test_contacts PRIVATE src)
maul2d_apply_flags(test_contacts)
add_test(NAME contacts COMMAND test_contacts)

add_executable(test_solver test/test_solver.c)
target_link_libraries(test_solver PRIVATE maul2d)
target_include_directories(test_solver PRIVATE src)
maul2d_apply_flags(test_solver)
add_test(NAME solver COMMAND test_solver)

option(MAUL2D_BENCHMARKS "Build the dual-backend harness (fetches Box2D)" OFF)
if(MAUL2D_BENCHMARKS)
    add_subdirectory(benchmark)
endif()
