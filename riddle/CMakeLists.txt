file(GLOB RIDDLE_SOURCES *.cpp)
file(GLOB RIDDLE_HEADERS *.h)

add_library(riddle SHARED ${RIDDLE_SOURCES})
GENERATE_EXPORT_HEADER(riddle)
target_include_directories(riddle PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> $<INSTALL_INTERFACE:>)
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
configure_package_config_file(riddleConfig.cmake.in riddleConfig.cmake INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/riddle PATH_VARS CMAKE_INSTALL_INCLUDEDIR)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/riddleConfigVersion.cmake VERSION 1.0.0 COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/riddleConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/riddleConfigVersion.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/riddle)