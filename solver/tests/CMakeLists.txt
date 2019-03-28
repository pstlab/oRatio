file(GLOB SOLVER_SOURCES test_solver.cpp ${CMAKE_SOURCE_DIR}/smt/*.cpp ${CMAKE_SOURCE_DIR}/smt/lra/*.cpp ${CMAKE_SOURCE_DIR}/smt/ov/*.cpp ${CMAKE_SOURCE_DIR}/riddle/*.cpp ${CMAKE_SOURCE_DIR}/core/*.cpp ${CMAKE_SOURCE_DIR}/solver/*.cpp ${CMAKE_SOURCE_DIR}/solver/types/*.cpp)

if(BUILD_GUI)
    list(APPEND SOLVER_SOURCES ${CMAKE_SOURCE_DIR}/java-api/src/main/cpp/socket_listener.cpp)
endif()

add_executable(solver_tests ${SOLVER_SOURCES})

if(BUILD_GUI AND WIN32)
    target_link_libraries(solver_tests wsock32 ws2_32 #[[-PROFILE]])
endif()