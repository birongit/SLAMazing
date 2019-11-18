include_directories(${CMAKE_SOURCE_DIR})

add_executable(test_solver solver.cpp)
target_link_libraries(test_solver gtest gmock gtest_main slam)
add_test(NAME test_solver COMMAND test_solver)
