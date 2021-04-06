file(GLOB RIDDLE_SOURCES *.cpp)

add_library(riddle SHARED ${RIDDLE_SOURCES})
target_include_directories(riddle PUBLIC ../smt ../smt/lra)
target_link_libraries(riddle PUBLIC smt)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()