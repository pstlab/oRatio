file(GLOB RBS_SOURCES *.cpp parser/*.cpp)
file(GLOB RBS_HEADERS *.h parser/*.h)

add_library(rbs SHARED ${RBS_SOURCES})
GENERATE_EXPORT_HEADER(rbs)
target_include_directories(rbs PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> $<INSTALL_INTERFACE:>)
target_link_libraries(rbs PUBLIC smt)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

if(MSVC)
    target_compile_options(rbs PRIVATE /W4)
else()
    target_compile_options(rbs PRIVATE -Wall -Wextra -pedantic)
endif()

install(TARGETS rbs LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES ${RBS_HEADERS} ${CMAKE_CURRENT_BINARY_DIR}/rbs_export.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rbs)
configure_package_config_file(rbsConfig.cmake.in rbsConfig.cmake INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rbs PATH_VARS CMAKE_INSTALL_INCLUDEDIR)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/rbsConfigVersion.cmake VERSION 1.0.0 COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/rbsConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/rbsConfigVersion.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rbs)