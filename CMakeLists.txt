cmake_minimum_required(VERSION 3.0.0)
project(oRatio VERSION 2.0.0)

set(CMAKE_CXX_STANDARD 17)

include(CTest)
enable_testing()

file(GLOB SOURCES main.cpp smt/*.cpp smt/lra/*.cpp smt/ov/*.cpp riddle/*.cpp core/*.cpp solver/*.cpp)
include_directories(smt smt/lra smt/ov riddle core solver solver/listeners)

configure_file(solver/init.rddl ${CMAKE_BINARY_DIR}/init.rddl COPYONLY)

set(HEURISTIC_TYPE "h_max" CACHE STRING "Heuristic type")
set_property(CACHE HEURISTIC_TYPE PROPERTY STRINGS "h_max" "h_add")

option(BUILD_GUI "Build GUI" OFF)

message(STATUS "Heuristic type: ${HEURISTIC_TYPE}")
message(STATUS "Build GUI:      ${BUILD_GUI}")

if(HEURISTIC_TYPE MATCHES "h_max")
    add_definitions(-DH_MAX)
elseif(HEURISTIC_TYPE MATCHES "h_add")
    add_definitions(-DH_ADD)
endif()

if(BUILD_GUI)
    add_definitions(-DBUILD_GUI)
    list(APPEND SOURCES solver/listeners/solver_listener.cpp solver/listeners/socket_listener.cpp)
endif()

add_executable(${PROJECT_NAME} ${SOURCES})

if(BUILD_GUI AND WIN32)
    target_link_libraries(${PROJECT_NAME} wsock32 ws2_32 #[[-PROFILE]])
endif()

add_subdirectory(smt/tests)
add_subdirectory(riddle/tests)
add_subdirectory(core/tests)
add_subdirectory(solver/tests)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
