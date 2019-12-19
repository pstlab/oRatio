cmake_minimum_required(VERSION 3.0.0)
project(oRatio VERSION 2.1.0)

find_package (JNI)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CTest)
enable_testing()

file(GLOB SOURCES main.cpp smt/*.cpp smt/lra/*.cpp smt/ov/*.cpp riddle/*.cpp core/*.cpp solver/*.cpp solver/types/*.cpp)
include_directories(smt smt/lra smt/ov riddle core solver solver/types java-api/src/main/cpp)

configure_file(solver/init.rddl ${CMAKE_BINARY_DIR}/init.rddl COPYONLY)

set(HEURISTIC_TYPE "h_max" CACHE STRING "Heuristic type")
set_property(CACHE HEURISTIC_TYPE PROPERTY STRINGS "h_max" "h_add")

set(HOST "127.0.0.1" CACHE STRING "Socket listener host")
set(PORT 1100 CACHE STRING "Socket listener port")
add_definitions(-DHOST="${HOST}" -DPORT=${PORT})

option(DEFERRABLE_FLAWS "Check for deferrable flaws" ON)
option(CHECK_CYCLES "Check for causal cycles and adds non-cyclicity constraints" OFF)
option(CHECK_GRAPH "Check the validity of the graph by applying the resolvers" OFF)
option(GRAPH_PRUNING "Prunes the causal graph before starting the search" ON)
option(CHECK_INCONSISTENCIES "Check inconsistencies at each step" OFF)
option(OPTIMIZE_FOR_NATIVE_ARCH "Optimize for native arch" OFF)
option(BUILD_GUI "Builds the GUI" OFF)
option(BUILD_JAVA_LIB "Builds the Java lib" OFF)

message(STATUS "Heuristic type:           ${HEURISTIC_TYPE}")
if(HEURISTIC_TYPE MATCHES "h_max")
    add_definitions(-DH_MAX)
elseif(HEURISTIC_TYPE MATCHES "h_add")
    add_definitions(-DH_ADD)
endif()

message(STATUS "Deferrable flaws:         ${DEFERRABLE_FLAWS}")
if(DEFERRABLE_FLAWS)
    add_definitions(-DDEFERRABLE_FLAWS)
endif()

message(STATUS "Check cycles:             ${CHECK_CYCLES}")
if(CHECK_CYCLES)
    add_definitions(-DCHECK_CYCLES)
endif()

message(STATUS "Check graph:              ${CHECK_GRAPH}")
if(CHECK_GRAPH)
    add_definitions(-DCHECK_GRAPH)
endif()

message(STATUS "Graph pruning:            ${GRAPH_PRUNING}")
if(GRAPH_PRUNING)
    add_definitions(-DGRAPH_PRUNING)
endif()

message(STATUS "Check inconsistencies:    ${CHECK_INCONSISTENCIES}")
if(CHECK_INCONSISTENCIES)
    add_definitions(-DCHECK_INCONSISTENCIES)
endif()

message(STATUS "Optimize for native arch: ${OPTIMIZE_FOR_NATIVE_ARCH}")
if (OPTIMIZE_FOR_NATIVE_ARCH)
    if(MSVC)
        message(STATUS "Using AVX2..")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -arch:AVX2")
    else()
        message(STATUS "Using native optimization..")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
    endif()
endif()

message(STATUS "Build GUI:                ${BUILD_GUI}")
if(BUILD_GUI)
    message(STATUS "Socket listener host:     ${HOST}")
    message(STATUS "Socket listener port:     ${PORT}")
    add_definitions(-DBUILD_GUI)
    list(APPEND SOURCES java-api/src/main/cpp/socket_listener.cpp)
endif()

add_executable(${PROJECT_NAME} ${SOURCES})

if(BUILD_GUI AND WIN32)
    target_link_libraries(${PROJECT_NAME} wsock32 ws2_32 #[[-PROFILE]])
endif()

message(STATUS "Build Java bindings:      ${BUILD_JAVA_LIB}")
if(BUILD_JAVA_LIB)
    add_definitions(-DBUILD_JAVA_LIB)
    include_directories(${JNI_INCLUDE_DIRS})
    list(APPEND SOURCES java-api/src/main/cpp/it_cnr_istc_oratio_Solver.cpp)
    add_library ("${PROJECT_NAME}-lib" SHARED ${SOURCES})
    if(BUILD_GUI AND WIN32)
        target_link_libraries("${PROJECT_NAME}-lib" wsock32 ws2_32 #[[-PROFILE]])
    endif()
endif()

add_subdirectory(smt/tests)
add_subdirectory(riddle/tests)
add_subdirectory(core/tests)
add_subdirectory(solver/tests)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
