cmake_minimum_required(VERSION 3.0.0)
project(oRatio VERSION 2.0.0)

set(CMAKE_CXX_STANDARD 17)

include(CTest)
enable_testing()

file(GLOB SOURCES main.cpp smt/*.cpp smt/lra/*.cpp smt/ov/*.cpp riddle/*.cpp core/*.cpp solver/*.cpp)
include_directories(smt smt/lra smt/ov riddle core solver)

set(EVAL_TYPE "h_max" CACHE STRING "Evaluation type")
set_property(CACHE EVAL_TYPE PROPERTY STRINGS "h_max" "h_add")

message(STATUS "Evaluation type: ${EVAL_TYPE}")

if(EVAL_TYPE MATCHES "h_max")
    add_definitions(-DH_MAX)
elseif(EVAL_TYPE MATCHES "h_add")
    add_definitions(-DH_ADD)
endif()

add_executable(${PROJECT_NAME} ${SOURCES})

add_subdirectory(smt/tests)
add_subdirectory(riddle/tests)
add_subdirectory(core/tests)
add_subdirectory(solver/tests)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
