file(GLOB SMT_SOURCES *.cpp arith/*.cpp arith/lra/*.cpp arith/dl/*.cpp ov/*.cpp)
file(GLOB SMT_HEADERS *.h arith/*.h arith/lra/*.h arith/dl/*.h ov/*.h)

add_library(smt SHARED ${SMT_SOURCES})
GENERATE_EXPORT_HEADER(smt)
target_include_directories(smt PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/arith $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/arith/lra $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/arith/dl $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/ov $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> $<INSTALL_INTERFACE:>)
target_link_libraries(smt PUBLIC json)

if(PARALLELIZE)
    target_include_directories(smt PUBLIC concurrent)
    target_link_libraries(smt PUBLIC concurrent)
endif()

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

if(MSVC)
    target_compile_options(smt PRIVATE /W4)
else()
    target_compile_options(smt PRIVATE -Wall -Wextra -pedantic)
endif()

install(TARGETS smt EXPORT smtTargets LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES ${SMT_HEADERS} ${CMAKE_CURRENT_BINARY_DIR}/smt_export.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/smt)
install(EXPORT smtTargets FILE smtTargets.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/smt NAMESPACE oRatio::)
configure_file(smtConfig.cmake.in smtConfig.cmake @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/smtConfig.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/smt)