file(GLOB CORE_SOURCES test_core.cpp ${CMAKE_SOURCE_DIR}/smt/*.cpp ${CMAKE_SOURCE_DIR}/smt/lra/*.cpp ${CMAKE_SOURCE_DIR}/smt/ov/*.cpp ${CMAKE_SOURCE_DIR}/riddle/*.cpp ${CMAKE_SOURCE_DIR}/core/*.cpp)

add_executable(core_lib_tests ${CORE_SOURCES})

add_test(NAME CORE_LibTest COMMAND core_lib_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})