add_executable(ql_tests test_ql.cpp)
target_include_directories(ql_tests PRIVATE ..)

add_test(NAME QLTest COMMAND ql_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})