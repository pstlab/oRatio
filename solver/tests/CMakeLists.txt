file(GLOB SOLVER_SOURCES test_solver.cpp ${CMAKE_SOURCE_DIR}/smt/*.cpp ${CMAKE_SOURCE_DIR}/smt/lra/*.cpp ${CMAKE_SOURCE_DIR}/smt/ov/*.cpp ${CMAKE_SOURCE_DIR}/riddle/*.cpp ${CMAKE_SOURCE_DIR}/core/*.cpp ${CMAKE_SOURCE_DIR}/solver/*.cpp)

add_executable(solver_tests ${SOLVER_SOURCES})

add_test(NAME SolverTest COMMAND solver_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})