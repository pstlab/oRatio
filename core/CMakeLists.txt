file(GLOB CORE_SOURCES *.cpp)
file(GLOB CORE_HEADERS ${CMAKE_CURRENT_BINARY_DIR}/*.h *.h)

add_library(core SHARED ${CORE_SOURCES})
GENERATE_EXPORT_HEADER(core)
target_include_directories(core PUBLIC ${CMAKE_CURRENT_BINARY_DIR} .)
target_link_libraries(core PUBLIC smt riddle)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

if(MSVC)
    target_compile_options(core PRIVATE /W4)
else()
    target_compile_options(core PRIVATE -Wall -Wextra -pedantic)
endif()

install(TARGETS core LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES ${CORE_HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/core)