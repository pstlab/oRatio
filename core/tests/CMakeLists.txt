file(GLOB CORE_SOURCES test_core.cpp ${CMAKE_SOURCE_DIR}/smt/*.cpp ${CMAKE_SOURCE_DIR}/smt/lra/*.cpp ${CMAKE_SOURCE_DIR}/smt/ov/*.cpp ${CMAKE_SOURCE_DIR}/riddle/*.cpp ${CMAKE_SOURCE_DIR}/core/*.cpp)

add_executable(core_lib_tests ${CORE_SOURCES})

if(OPTIMIZE_FOR_NATIVE_ARCH)
    target_link_libraries(core_lib_tests PRIVATE Threads::Threads)
endif()

add_test(NAME CORE_LibTest COMMAND core_lib_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})