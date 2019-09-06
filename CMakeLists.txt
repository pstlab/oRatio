cmake_minimum_required(VERSION 3.0.0)
project(oRatio VERSION 2.1.0)

set(CMAKE_CXX_STANDARD 17)

include(CTest)
enable_testing()

file(GLOB SOURCES main.cpp smt/*.cpp smt/lra/*.cpp smt/ov/*.cpp riddle/*.cpp core/*.cpp solver/*.cpp solver/types/*.cpp)
include_directories(smt smt/lra smt/ov riddle core solver solver/types java-api/src/main/cpp)

configure_file(solver/init.rddl ${CMAKE_BINARY_DIR}/init.rddl COPYONLY)

set(HEURISTIC_TYPE "h_max" CACHE STRING "Heuristic type")
set_property(CACHE HEURISTIC_TYPE PROPERTY STRINGS "h_max" "h_add")
set(MIN_ACCURACY 1 CACHE STRING "Min heuristic accuracy")
set(MAX_ACCURACY 2 CACHE STRING "Max heuristic accuracy")
add_definitions(-DMIN_ACCURACY=${MIN_ACCURACY} -DMAX_ACCURACY=${MAX_ACCURACY})

set(HOST "127.0.0.1" CACHE STRING "Socket listener host")
set(PORT 1100 CACHE STRING "Socket listener port")
add_definitions(-DHOST="${HOST}" -DPORT=${PORT})

option(DEFERRABLE_FLAWS "Check for deferrable flaws" ON)
option(CHECK_CYCLES "Check for causal cycles and adds non-cyclicity constraints" ON)
option(CHECK_GRAPH "Check the validity of the graph by applying the resolvers" OFF)
option(GRAPH_PRUNING "Prunes the causal graph before starting the search" OFF)
option(CHECK_INCONSISTENCIES "Check inconsistencies at each step" OFF)
option(BUILD_GUI "Builds the GUI" OFF)

message(STATUS "Heuristic type:         ${HEURISTIC_TYPE}")
message(STATUS "Min heuristic accuracy: ${MIN_ACCURACY}")
message(STATUS "Max heuristic accuracy: ${MAX_ACCURACY}")
message(STATUS "Deferrable flaws:       ${DEFERRABLE_FLAWS}")
message(STATUS "Check cycles:           ${CHECK_CYCLES}")
message(STATUS "Check graph:            ${CHECK_GRAPH}")
message(STATUS "Graph pruning:          ${GRAPH_PRUNING}")
message(STATUS "Check inconsistencies:  ${CHECK_INCONSISTENCIES}")
message(STATUS "Build GUI:              ${BUILD_GUI}")

if(BUILD_GUI)
    message(STATUS "Socket listener host:   ${HOST}")
    message(STATUS "Socket listener port:   ${PORT}")
endif()

if(HEURISTIC_TYPE MATCHES "h_max")
    add_definitions(-DH_MAX)
elseif(HEURISTIC_TYPE MATCHES "h_add")
    add_definitions(-DH_ADD)
endif()

if(DEFERRABLE_FLAWS)
    add_definitions(-DDEFERRABLE_FLAWS)
endif()

if(CHECK_CYCLES)
    add_definitions(-DCHECK_CYCLES)
endif()

if(CHECK_GRAPH)
    add_definitions(-DCHECK_GRAPH)
endif()

if(GRAPH_PRUNING)
    add_definitions(-DGRAPH_PRUNING)
endif()

if(CHECK_INCONSISTENCIES)
    add_definitions(-DCHECK_INCONSISTENCIES)
endif()

if(BUILD_GUI)
    add_definitions(-DBUILD_GUI)
    list(APPEND SOURCES java-api/src/main/cpp/socket_listener.cpp)
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
