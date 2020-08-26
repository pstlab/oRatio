file(GLOB SAT_SOURCES test_sat.cpp ${CMAKE_SOURCE_DIR}/smt/*.cpp)
file(GLOB LRA_SOURCES test_lra.cpp ${CMAKE_SOURCE_DIR}/smt/*.cpp ${CMAKE_SOURCE_DIR}/smt/lra/*.cpp)

add_executable(sat_lib_tests ${SAT_SOURCES})
add_executable(lra_lib_tests ${LRA_SOURCES})

add_test(NAME SAT_LibTest COMMAND sat_lib_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME LRA_LibTest COMMAND lra_lib_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})