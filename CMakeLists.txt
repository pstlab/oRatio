cmake_minimum_required(VERSION 3.0.0)
project(oRatio VERSION 2.2.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CTest)
enable_testing()

file(GLOB SOURCES main.cpp smt/*.cpp smt/lra/*.cpp smt/ov/*.cpp smt/dl/*.cpp riddle/*.cpp core/*.cpp solver/*.cpp solver/types/*.cpp)
include_directories(smt smt/parallel smt/lra smt/ov smt/dl riddle core solver solver/types java-api/src/main/cpp)

add_executable(${PROJECT_NAME} ${SOURCES})

add_subdirectory(smt/tests)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
