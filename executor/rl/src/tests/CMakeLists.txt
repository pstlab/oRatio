file(GLOB QL_SOURCES test_ql.cpp ${CMAKE_SOURCE_DIR}/src/*.cpp)

add_executable(ql_tests ${QL_SOURCES})
target_link_libraries(ql_tests "${TORCH_LIBRARIES}")

add_test(NAME QLTest COMMAND ql_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})