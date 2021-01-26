cmake_minimum_required(VERSION 3.0.0)
project(oRatio VERSION 2.9.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(JNI)
find_package(Threads)

add_subdirectory(smt)
add_subdirectory(riddle)
add_subdirectory(core)
add_subdirectory(solver)

add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE solver core smt smt/lra smt/ov smt/dl)
target_link_libraries(${PROJECT_NAME} PRIVATE solver)

include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
