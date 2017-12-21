add_executable( smt_lib_tests tests.cpp ${CMAKE_SOURCE_DIR}/smt-lib/sat_core.cpp )

add_test( NAME Test0 COMMAND smt_lib_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR} )