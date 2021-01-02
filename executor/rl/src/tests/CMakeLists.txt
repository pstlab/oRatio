file(GLOB QL_SOURCES test_ql.cpp ${CMAKE_SOURCE_DIR}/src/*.cpp)
file(GLOB DQL_SOURCES test_dql.cpp ${CMAKE_SOURCE_DIR}/src/*.cpp)

add_executable(ql_tests ${QL_SOURCES})
target_link_libraries(ql_tests "${TORCH_LIBRARIES}")

add_executable(dql_tests ${DQL_SOURCES})
target_link_libraries(dql_tests "${TORCH_LIBRARIES}")

add_test(NAME QLTest COMMAND ql_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME DQLTest COMMAND dql_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

if (MSVC)
  file(GLOB TORCH_DLLS "${TORCH_INSTALL_PREFIX}/lib/*.dll")
  add_custom_command(TARGET ql_tests POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different ${TORCH_DLLS} $<TARGET_FILE_DIR:ql_tests>)
endif (MSVC)