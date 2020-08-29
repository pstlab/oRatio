file(GLOB LEXER_SOURCES test_lexer.cpp ${CMAKE_SOURCE_DIR}/riddle/lexer.cpp ${CMAKE_SOURCE_DIR}/smt/lra/rational.cpp)
file(GLOB PARSER_SOURCES test_parser.cpp ${CMAKE_SOURCE_DIR}/riddle/*.cpp ${CMAKE_SOURCE_DIR}/smt/lra/rational.cpp)

add_executable(lexer_lib_tests ${LEXER_SOURCES})
add_executable(parser_lib_tests ${PARSER_SOURCES})

add_test(NAME RIDDLE_LexerTest COMMAND lexer_lib_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME RIDDLE_ParserTest COMMAND parser_lib_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})