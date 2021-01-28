add_executable(lexer_lib_tests test_lexer.cpp)
target_include_directories(lexer_lib_tests PRIVATE .. ../../smt ../../smt/lra)
target_link_libraries(lexer_lib_tests PRIVATE riddle smt)

add_executable(parser_lib_tests test_parser.cpp)
target_include_directories(parser_lib_tests PRIVATE .. ../../smt ../../smt/lra)
target_link_libraries(parser_lib_tests PRIVATE riddle smt)

add_test(NAME RIDDLE_LexerTest COMMAND lexer_lib_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME RIDDLE_ParserTest COMMAND parser_lib_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})