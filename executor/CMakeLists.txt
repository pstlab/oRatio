find_package(Threads REQUIRED)

file(GLOB EXECUTOR_SOURCES *.cpp)
file(GLOB EXECUTOR_HEADERS *.h)

add_library(executor SHARED ${EXECUTOR_SOURCES})
GENERATE_EXPORT_HEADER(executor)
target_include_directories(executor PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> $<INSTALL_INTERFACE:>)
target_link_libraries(executor PUBLIC solver)

if(MSVC)
    target_compile_options(executor PRIVATE /W4)
else()
    target_compile_options(executor PRIVATE -Wall -Wextra -pedantic)
endif()

install(TARGETS executor EXPORT executorTargets LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES ${EXECUTOR_HEADERS} ${CMAKE_CURRENT_BINARY_DIR}/executor_export.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/executor)
install(EXPORT executorTargets FILE executorTargets.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/executor NAMESPACE oRatio::)
configure_file(executorConfig.cmake.in executorConfig.cmake @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/executorConfig.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/executor)