file( GLOB SMT_SOURCES tests.cpp ${CMAKE_SOURCE_DIR}/smt-lib/*.cpp ${CMAKE_SOURCE_DIR}/smt-lib/lra/*.cpp )
add_executable( smt_lib_tests ${SMT_SOURCES} )

add_test( NAME Test0 COMMAND smt_lib_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR} )