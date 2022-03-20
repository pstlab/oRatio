find_package(Threads REQUIRED)

file(GLOB CONCURRENT_SOURCES *.cpp)
file(GLOB CONCURRENT_HEADERS *.h)

add_library(concurrent SHARED ${CONCURRENT_SOURCES})
GENERATE_EXPORT_HEADER(concurrent)
target_include_directories(concurrent PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> $<INSTALL_INTERFACE:>)
target_link_libraries(concurrent PRIVATE Threads::Threads)

install(TARGETS concurrent EXPORT concurrentTargets LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES ${CONCURRENT_HEADERS} ${CMAKE_CURRENT_BINARY_DIR}/concurrent_export.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/concurrent)
install(EXPORT concurrentTargets FILE concurrentConfig.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/concurrent NAMESPACE oRatio::)