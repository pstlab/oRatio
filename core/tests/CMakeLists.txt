file(GLOB CORE_SOURCES test_core.cpp ${CMAKE_SOURCE_DIR}/smt/*.cpp ${CMAKE_SOURCE_DIR}/smt/lra/*.cpp ${CMAKE_SOURCE_DIR}/smt/ov/*.cpp ${CMAKE_SOURCE_DIR}/smt/dl/*.cpp ${CMAKE_SOURCE_DIR}/riddle/*.cpp ${CMAKE_SOURCE_DIR}/core/*.cpp)

add_executable(core_lib_tests test_core.cpp)
target_include_directories(core_lib_tests PRIVATE .. ../../smt ../../smt/lra ../../smt/ov ../../smt/dl)
target_link_libraries(core_lib_tests PRIVATE core smt)

add_test(NAME CORE_LibTest COMMAND core_lib_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})