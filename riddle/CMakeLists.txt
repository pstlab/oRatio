file(GLOB RIDDLE_SOURCES *.cpp)

add_library(riddle ${RIDDLE_SOURCES})
target_include_directories(riddle PRIVATE ../smt ../smt/lra)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()