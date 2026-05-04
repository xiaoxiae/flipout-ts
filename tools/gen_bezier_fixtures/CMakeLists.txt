cmake_minimum_required(VERSION 3.14)
project(gen_bezier_fixtures CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# geometry-central as a static-library subproject. We never install it; its
# CMake target is just consumed by our executable.
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
add_subdirectory(extern/geometry-central EXCLUDE_FROM_ALL)

add_executable(gen_bezier_fixtures main.cpp)
target_link_libraries(gen_bezier_fixtures PRIVATE geometry-central)
target_compile_options(gen_bezier_fixtures PRIVATE -Wall -Wextra)
