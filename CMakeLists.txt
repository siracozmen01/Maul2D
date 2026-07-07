cmake_minimum_required(VERSION 3.22)
project(maul2d LANGUAGES C)

option(MAUL2D_SANITIZE "Build with ASAN/UBSAN" OFF)

include(cmake/CompilerFlags.cmake)

find_package(Threads REQUIRED)

add_library(maul2d STATIC
    src/core.c
    src/ccd.c
    src/distance.c
    src/dynamic_tree.c
    src/island.c
    src/journal.c
    src/particle.c
    src/query.c
    src/platform_thread.c
    src/draw.c
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

add_executable(test_islands test/test_islands.c)
target_link_libraries(test_islands PRIVATE maul2d)
target_include_directories(test_islands PRIVATE src)
maul2d_apply_flags(test_islands)
add_test(NAME islands COMMAND test_islands)

add_executable(test_ccd test/test_ccd.c)
target_link_libraries(test_ccd PRIVATE maul2d)
target_include_directories(test_ccd PRIVATE src)
maul2d_apply_flags(test_ccd)
add_test(NAME ccd COMMAND test_ccd)

add_executable(test_events test/test_events.c)
target_link_libraries(test_events PRIVATE maul2d)
target_include_directories(test_events PRIVATE src)
maul2d_apply_flags(test_events)
add_test(NAME events COMMAND test_events)

add_executable(test_joints test/test_joints.c)
target_link_libraries(test_joints PRIVATE maul2d)
target_include_directories(test_joints PRIVATE src)
maul2d_apply_flags(test_joints)
add_test(NAME joints COMMAND test_joints)

add_executable(test_journal test/test_journal.c)
target_link_libraries(test_journal PRIVATE maul2d)
target_include_directories(test_journal PRIVATE src)
maul2d_apply_flags(test_journal)
add_test(NAME journal COMMAND test_journal)

add_executable(test_queries test/test_queries.c)
target_link_libraries(test_queries PRIVATE maul2d)
target_include_directories(test_queries PRIVATE src)
maul2d_apply_flags(test_queries)
add_test(NAME queries COMMAND test_queries)

add_executable(test_redteam test/test_redteam.c)
target_link_libraries(test_redteam PRIVATE maul2d)
target_include_directories(test_redteam PRIVATE src)
maul2d_apply_flags(test_redteam)
add_test(NAME redteam COMMAND test_redteam)

add_executable(test_fuzz test/test_fuzz.c)
target_link_libraries(test_fuzz PRIVATE maul2d)
maul2d_apply_flags(test_fuzz)
add_test(NAME fuzz COMMAND test_fuzz)

add_executable(test_soak test/test_soak.c)
target_link_libraries(test_soak PRIVATE maul2d)
maul2d_apply_flags(test_soak)
# Deliberately not registered with ctest: the soak runs deep in the
# weekly scheduled CI job and shallow by hand (test_soak [steps]).

add_executable(test_fluid test/test_fluid.c)
target_include_directories(test_fluid PRIVATE src)
target_link_libraries(test_fluid PRIVATE maul2d)
maul2d_apply_flags(test_fluid)
add_test(NAME fluid COMMAND test_fluid)

add_executable(test_threads test/test_threads.c)
target_link_libraries(test_threads PRIVATE maul2d)
target_include_directories(test_threads PRIVATE src)
maul2d_apply_flags(test_threads)
add_test(NAME threads COMMAND test_threads)

option(MAUL2D_SAMPLES "Build the sample programs" ON)
if(MAUL2D_SAMPLES)
    add_subdirectory(samples)
endif()

option(MAUL2D_BENCHMARKS "Build the dual-backend harness (fetches Box2D)" OFF)
option(MAUL2D_TESTBED "Build the interactive testbed (fetches raylib)" OFF)
if(MAUL2D_TESTBED)
    add_subdirectory(testbed)
endif()

if(MAUL2D_BENCHMARKS)
    add_subdirectory(benchmark)
endif()
