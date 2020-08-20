file(GLOB SAT_SOURCES test_sat.cpp ${CMAKE_SOURCE_DIR}/smt/*.cpp)

add_executable(sat_lib_tests ${SAT_SOURCES})

add_test(NAME SAT_LibTest COMMAND sat_lib_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})