cmake_minimum_required(VERSION 3.9)
project(slam)
set(CMAKE_CXX_STANDARD 14)

add_subdirectory(extern/csparse)
add_subdirectory(src)

add_executable(slam_demo
               main.cpp
               )

target_link_libraries(slam_demo slam)

# Tests
add_subdirectory(extern/googletest)
enable_testing()
add_subdirectory(test)

# Format
include(cmake/ClangFormat.cmake)
