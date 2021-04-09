file(GLOB RIDDLE_SOURCES *.cpp)

add_library(riddle SHARED ${RIDDLE_SOURCES})
GENERATE_EXPORT_HEADER(riddle)
target_include_directories(riddle PUBLIC ${CMAKE_CURRENT_BINARY_DIR} ../smt ../smt/lra)
target_link_libraries(riddle PUBLIC smt)
install(TARGETS riddle)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()