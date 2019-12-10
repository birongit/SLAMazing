set(SOURCE_FILES
    graph.h
    graph.cpp
    solver.h
    solver.cpp
    utils.h
    utils.cpp
    )

include_directories(${CMAKE_SOURCE_DIR}/extern)

add_library(slam ${SOURCE_FILES})
target_link_libraries(slam PUBLIC csparse)
