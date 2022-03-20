find_package(Threads REQUIRED)

file(GLOB CONCURRENT_SOURCES *.cpp)
file(GLOB CONCURRENT_HEADERS *.h)

add_library(concurrent SHARED ${CONCURRENT_SOURCES})
GENERATE_EXPORT_HEADER(concurrent)
target_include_directories(concurrent PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> $<INSTALL_INTERFACE:>)
target_link_libraries(concurrent PRIVATE Threads::Threads)

install(TARGETS concurrent LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES ${CONCURRENT_HEADERS} ${CMAKE_CURRENT_BINARY_DIR}/concurrent_export.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/concurrent)
configure_package_config_file(concurrentConfig.cmake.in concurrentConfig.cmake INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/concurrent PATH_VARS CMAKE_INSTALL_INCLUDEDIR)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/concurrentConfigVersion.cmake VERSION 1.0.0 COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/concurrentConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/concurrentConfigVersion.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/concurrent)