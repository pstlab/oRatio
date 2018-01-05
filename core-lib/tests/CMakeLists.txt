file( GLOB PARSER_SOURCES tests.cpp ${CMAKE_SOURCE_DIR}/smt-lib/*.cpp ${CMAKE_SOURCE_DIR}/smt-lib/lra/*.cpp ${CMAKE_SOURCE_DIR}/smt-lib/ov/*.cpp ${CMAKE_SOURCE_DIR}/core-lib/*.cpp ${CMAKE_SOURCE_DIR}/core-lib/parser/*.cpp ${CMAKE_SOURCE_DIR}/core-lib/parser/declarations/*.cpp ${CMAKE_SOURCE_DIR}/core-lib/parser/statements/*.cpp ${CMAKE_SOURCE_DIR}/core-lib/parser/expressions/*.cpp )

add_executable( parser_tests ${PARSER_SOURCES} )

add_test( NAME ParserTest COMMAND parser_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR} )