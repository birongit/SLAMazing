set(SOURCE_FILES
    solver.h
    solver.cpp
    )

include_directories(${CMAKE_SOURCE_DIR}/extern)

add_library(slam ${SOURCE_FILES})
target_link_libraries(slam PUBLIC csparse)
