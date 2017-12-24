file( GLOB PARSER_SOURCES tests.cpp ${CMAKE_SOURCE_DIR}/core-lib/parser/*.cpp ${CMAKE_SOURCE_DIR}/smt-lib/lra/rational.cpp )
add_executable( parser_tests ${PARSER_SOURCES} )

add_test( NAME ParserTest COMMAND parser_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR} )