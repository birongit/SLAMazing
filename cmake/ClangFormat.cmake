file(GLOB_RECURSE ALL_SOURCE_FILES *.cpp *.h *.cu *.hpp *.c *.cc)

# don't include build and 3rd party libs
set(CLANG_FORMAT_EXCLUDE_PATTERNS
    ${CLANG_FORMAT_EXCLUDE_PATTERNS}
    "build/"
    "extern/"
    "/CMakeFiles/"
    "cmake")

# get all project files
foreach (SOURCE_FILE ${ALL_SOURCE_FILES})
    foreach (EXCLUDE_PATTERN ${CLANG_FORMAT_EXCLUDE_PATTERNS})
        string(FIND ${SOURCE_FILE} ${EXCLUDE_PATTERN} EXCLUDE_FOUND)
        if (NOT ${EXCLUDE_FOUND} EQUAL -1)
            list(REMOVE_ITEM ALL_SOURCE_FILES ${SOURCE_FILE})
        endif ()
    endforeach ()
endforeach ()

find_program(CLANG_FORMAT NAMES clang-format)
if (CLANG_FORMAT)
    MESSAGE(STATUS "Adding clang-format target.")
    add_custom_target(
            clang-format
            COMMAND ${CLANG_FORMAT}
            -i
            -style=llvm
            ${ALL_SOURCE_FILES}
    )
endif ()
