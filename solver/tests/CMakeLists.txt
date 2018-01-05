file( GLOB SOLVER_SOURCES tests.cpp ${CMAKE_SOURCE_DIR}/smt-lib/*.cpp ${CMAKE_SOURCE_DIR}/smt-lib/lra/*.cpp ${CMAKE_SOURCE_DIR}/smt-lib/ov/*.cpp ${CMAKE_SOURCE_DIR}/core-lib/*.cpp ${CMAKE_SOURCE_DIR}/core-lib/parser/*.cpp ${CMAKE_SOURCE_DIR}/core-lib/parser/declarations/*.cpp ${CMAKE_SOURCE_DIR}/core-lib/parser/statements/*.cpp ${CMAKE_SOURCE_DIR}/core-lib/parser/expressions/*.cpp ${CMAKE_SOURCE_DIR}/solver/*.cpp ${CMAKE_SOURCE_DIR}/solver/types/*.cpp ${CMAKE_SOURCE_DIR}/solver/listener/*.cpp )

add_executable( solver_tests ${SOLVER_SOURCES} )

if(WIN32)
  target_link_libraries( solver_tests wsock32 ws2_32 )
endif()

add_test( NAME TestCG0 COMMAND ${PROJECT_NAME} "${CMAKE_SOURCE_DIR}/examples/example_0.rddl" "solution.json" WORKING_DIRECTORY ${CMAKE_BINARY_DIR} )
add_test( NAME TestCG1 COMMAND ${PROJECT_NAME} "${CMAKE_SOURCE_DIR}/examples/example_1.rddl" "solution.json" WORKING_DIRECTORY ${CMAKE_BINARY_DIR} )
add_test( NAME TestCG2 COMMAND ${PROJECT_NAME} "${CMAKE_SOURCE_DIR}/examples/example_2.rddl" "solution.json" WORKING_DIRECTORY ${CMAKE_BINARY_DIR} )
add_test( NAME TestCG3 COMMAND ${PROJECT_NAME} "${CMAKE_SOURCE_DIR}/examples/test_heuristic_failure_0.rddl" "solution.json" WORKING_DIRECTORY ${CMAKE_BINARY_DIR} )
add_test( NAME TestCG4 COMMAND ${PROJECT_NAME} "${CMAKE_SOURCE_DIR}/examples/test_heuristic_failure_1.rddl" "solution.json" WORKING_DIRECTORY ${CMAKE_BINARY_DIR} )
set_tests_properties( TestCG2 TestCG3 TestCG4 PROPERTIES WILL_FAIL TRUE )