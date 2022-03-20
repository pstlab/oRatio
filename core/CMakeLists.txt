file(GLOB CORE_SOURCES *.cpp)
file(GLOB CORE_HEADERS *.h)

add_library(core SHARED ${CORE_SOURCES})
GENERATE_EXPORT_HEADER(core)
target_include_directories(core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> $<INSTALL_INTERFACE:>)
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
install(FILES ${CORE_HEADERS} ${CMAKE_CURRENT_BINARY_DIR}/core_export.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/core)
configure_package_config_file(coreConfig.cmake.in coreConfig.cmake INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/core PATH_VARS CMAKE_INSTALL_INCLUDEDIR)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/coreConfigVersion.cmake VERSION 1.0.0 COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/coreConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/coreConfigVersion.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/core)