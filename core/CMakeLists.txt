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

install(TARGETS core EXPORT coreTargets LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES ${CORE_HEADERS} ${CMAKE_CURRENT_BINARY_DIR}/core_export.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/core)
install(EXPORT coreTargets FILE coreConfig.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/core NAMESPACE oRatio::)