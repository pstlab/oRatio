file(GLOB SOLVER_SOURCES test_solver.cpp ${CMAKE_SOURCE_DIR}/smt/*.cpp ${CMAKE_SOURCE_DIR}/smt/lra/*.cpp ${CMAKE_SOURCE_DIR}/smt/ov/*.cpp ${CMAKE_SOURCE_DIR}/smt/dl/*.cpp ${CMAKE_SOURCE_DIR}/riddle/*.cpp ${CMAKE_SOURCE_DIR}/core/*.cpp ${CMAKE_SOURCE_DIR}/solver/*.cpp ${CMAKE_SOURCE_DIR}/solver/types/*.cpp)

if(BUILD_GUI)
    list(APPEND SOLVER_SOURCES ${CMAKE_SOURCE_DIR}/api/java/src/main/cpp/socket_listener.cpp)
endif()

add_executable(solver_tests ${SOLVER_SOURCES})

if(BUILD_GUI AND WIN32)
    target_link_libraries(solver_tests PRIVATE wsock32 ws2_32 #[[-PROFILE]])
endif()

add_test(NAME SolverTest00 COMMAND solver_tests "${CMAKE_SOURCE_DIR}/examples/core/example_00.rddl" "solution.json" WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME SolverTest01 COMMAND solver_tests "${CMAKE_SOURCE_DIR}/examples/core/example_01.rddl" "solution.json" WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME SolverTest02 COMMAND solver_tests "${CMAKE_SOURCE_DIR}/examples/core/example_02.rddl" "solution.json" WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME SolverTest03 COMMAND solver_tests "${CMAKE_SOURCE_DIR}/examples/core/example_03.rddl" "solution.json" WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME SolverTest04 COMMAND solver_tests "${CMAKE_SOURCE_DIR}/examples/core/example_04.rddl" "solution.json" WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
set_tests_properties(SolverTest02 PROPERTIES WILL_FAIL TRUE)