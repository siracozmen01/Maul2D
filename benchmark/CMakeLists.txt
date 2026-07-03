# Dual-backend harness. Off by default so the core library build stays
# dependency-free; enabling it fetches Box2D v3 (the reference backend).

include(FetchContent)

set(BOX2D_SAMPLES OFF CACHE BOOL "" FORCE)
set(BOX2D_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(BOX2D_DOCS OFF CACHE BOOL "" FORCE)
set(BOX2D_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BOX2D_VALIDATE OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    box2d
    GIT_REPOSITORY https://github.com/erincatto/box2d.git
    GIT_TAG v3.1.1)
FetchContent_MakeAvailable(box2d)

add_executable(harness harness.c)
target_link_libraries(harness PRIVATE maul2d box2d)
maul2d_apply_flags(harness)
if(NOT MSVC)
    target_link_libraries(harness PRIVATE m)
endif()

if(NOT MSVC)
    # Box2D's public headers trip -Wdouble-promotion; third-party headers
    # are exempt from our warning wall (our own code is not).
    target_compile_options(harness PRIVATE -Wno-double-promotion)
endif()
