file(GLOB RIDDLE_SOURCES *.cpp)

add_library(riddle STATIC ${RIDDLE_SOURCES})
target_include_directories(riddle PUBLIC ../smt ../smt/lra)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()