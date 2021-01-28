add_executable(dql_tests test_dql.cpp)
target_include_directories(dql_tests PRIVATE ..)
target_link_libraries(dql_tests "${TORCH_LIBRARIES}")

add_test(NAME DQLTest COMMAND dql_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})