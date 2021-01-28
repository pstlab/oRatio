cmake_minimum_required(VERSION 3.0.0)
project(oRatio VERSION 2.9.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(VERBOSE_LOG "Makes log more verbose" OFF)

set(HOST "127.0.0.1" CACHE STRING "Solver listener host")
set(SOLVER_PORT 1100 CACHE STRING "Solver listener port")
option(BUILD_SOLVER_LISTENER "Builds the GUI" OFF)

set(EXECUTOR_PORT 1101 CACHE STRING "Executor listener port")
option(BUILD_EXECUTOR_LISTENER "Builds the executor listener" OFF)

option(BUILD_JAVA_SOLVER_API "Builds the Java Solver API" OFF)

option(OPTIMIZE_FOR_NATIVE_ARCH "Optimize for native arch" OFF)

message(STATUS "Verbose log:                ${VERBOSE_LOG}")
if(VERBOSE_LOG)
    add_compile_definitions(VERBOSE_LOG)
endif()

message(STATUS "Build solver listener:      ${BUILD_SOLVER_LISTENER}")
if(BUILD_SOLVER_LISTENER)
    message(STATUS "Solver listener host:       ${HOST}")
    message(STATUS "Solver listener port:       ${SOLVER_PORT}")
    add_subdirectory(api/java/src/main/cpp/listener/solver)
endif()

message(STATUS "Build executor listener:    ${BUILD_EXECUTOR_LISTENER}")
if(BUILD_EXECUTOR_LISTENER)
    message(STATUS "Executor listener port:     ${EXECUTOR_PORT}")
    add_subdirectory(api/java/src/main/cpp/listener/executor)
endif()

message(STATUS "Build Java Solver bindings: ${BUILD_JAVA_SOLVER_API}")
if(BUILD_JAVA_SOLVER_API)
    add_subdirectory(api/java/src/main/cpp/api/solver)
endif()

message(STATUS "Optimize for native arch:   ${OPTIMIZE_FOR_NATIVE_ARCH}")
if(OPTIMIZE_FOR_NATIVE_ARCH)
    if(MSVC)
        message(STATUS "Using AVX2..")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -arch:AVX2")
    else()
        message(STATUS "Using native optimization..")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
    endif()
endif()

include(CTest)
enable_testing()

add_subdirectory(smt)
add_subdirectory(riddle)
add_subdirectory(core)
add_subdirectory(solver)

add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE solver core smt smt/lra smt/ov smt/dl)
target_link_libraries(${PROJECT_NAME} PRIVATE solver)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
