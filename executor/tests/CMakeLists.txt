file(GLOB EXECUTOR_SOURCES test_executor.cpp ${CMAKE_SOURCE_DIR}/smt/*.cpp ${CMAKE_SOURCE_DIR}/smt/lra/*.cpp ${CMAKE_SOURCE_DIR}/smt/ov/*.cpp ${CMAKE_SOURCE_DIR}/smt/dl/*.cpp ${CMAKE_SOURCE_DIR}/riddle/*.cpp ${CMAKE_SOURCE_DIR}/core/*.cpp ${CMAKE_SOURCE_DIR}/solver/*.cpp ${CMAKE_SOURCE_DIR}/solver/types/*.cpp ${CMAKE_SOURCE_DIR}/executor/*.cpp)

if(BUILD_GUI)
    add_compile_definitions(BUILD_GUI BUILD_LISTENERS)
    list(APPEND EXECUTOR_SOURCES ${CMAKE_SOURCE_DIR}/api/java/src/main/cpp/socket_listener.cpp)
endif()

add_executable(executor_tests ${EXECUTOR_SOURCES})

if(BUILD_GUI AND WIN32)
    target_link_libraries(executor_tests PRIVATE wsock32 ws2_32 #[[-PROFILE]])
endif()

add_test(NAME SVTestExecutor00 COMMAND executor_tests "${CMAKE_SOURCE_DIR}/examples/sv/sv_0.rddl" "solution.json" WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
