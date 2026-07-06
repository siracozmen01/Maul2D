# The testbed is a viewer, not simulation: it may use any libraries it
# likes because none of its code is inside the determinism contract.
# raylib is fetched and pinned; the engine itself stays zero-dependency.

include(FetchContent)

set(FETCHCONTENT_QUIET ON)
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
    GIT_SHALLOW TRUE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(raylib)

add_executable(testbed main.c)
target_link_libraries(testbed PRIVATE maul2d raylib)
set_target_properties(testbed PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED ON)
if(NOT MSVC)
    target_compile_options(testbed PRIVATE -Wall -Wextra)
endif()
