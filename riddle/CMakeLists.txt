file(GLOB RIDDLE_SOURCES *.cpp)
file(GLOB RIDDLE_HEADERS *.h)

add_library(riddle SHARED ${RIDDLE_SOURCES})
GENERATE_EXPORT_HEADER(riddle)
target_include_directories(riddle PUBLIC ${CMAKE_CURRENT_BINARY_DIR} .)
target_link_libraries(riddle PUBLIC smt)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

if(MSVC)
    target_compile_options(riddle PRIVATE /W4)
else()
    target_compile_options(riddle PRIVATE -Wall -Wextra -pedantic)
endif()

install(TARGETS riddle LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES ${RIDDLE_HEADERS} ${CMAKE_CURRENT_BINARY_DIR}/riddle_export.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/riddle)