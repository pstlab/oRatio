file(GLOB RIDDLE_SOURCES *.cpp)

add_library(riddle SHARED ${RIDDLE_SOURCES})
GENERATE_EXPORT_HEADER(riddle)
target_include_directories(riddle PUBLIC ../smt ../smt/lra ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(riddle PUBLIC smt)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()