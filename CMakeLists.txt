cmake_minimum_required(VERSION 3.0.0)
project(oRatio VERSION 2.0.0)

include(CTest)
enable_testing()

file(GLOB SOURCES main.cpp smt/*.cpp smt/lra/*.cpp smt/ov/*.cpp riddle/*.cpp core/*.cpp)
include_directories(smt smt/lra smt/ov riddle core)

add_executable(${PROJECT_NAME} ${SOURCES})

add_subdirectory(smt/tests)
add_subdirectory(riddle/tests)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
