cmake_minimum_required(VERSION 3.0.0)
project(oRatio VERSION 2.9.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include (GenerateExportHeader)

option(VERBOSE_LOG "Makes log more verbose" OFF)
option(PARALLELIZE "Performs some activities in parallel" OFF)

option(BUILD_EXECUTOR "Builds the executor" OFF)

option(BUILD_JAVA_API "Builds the Java API" OFF)
option(BUILD_SERVER "Builds the server" OFF)

option(OPTIMIZE_FOR_NATIVE_ARCH "Optimize for native arch" OFF)

include(CTest)
enable_testing()

add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE solver core smt smt/lra smt/ov smt/dl)
target_link_libraries(${PROJECT_NAME} PRIVATE solver)
install(TARGETS ${PROJECT_NAME})

add_subdirectory(smt)
add_subdirectory(smt/json)
add_subdirectory(riddle)
add_subdirectory(core)
add_subdirectory(solver)

message(STATUS "Parallelize:              ${PARALLELIZE}")
if(PARALLELIZE)
    add_subdirectory(smt/concurrent)
    target_compile_definitions(smt PUBLIC PARALLELIZE)
endif()

message(STATUS "Build executor:           ${BUILD_EXECUTOR}")
if(BUILD_EXECUTOR)
    add_subdirectory(executor)
    target_compile_definitions(solver PUBLIC BUILD_EXECUTOR)
endif()

message(STATUS "Build Java API:           ${BUILD_JAVA_API}")
if(BUILD_JAVA_API)
    add_subdirectory(api/java/src/main/cpp/solver)
endif()

message(STATUS "Build server:             ${BUILD_SERVER}")
if(BUILD_SERVER)
    add_subdirectory(gui/src/main/cpp)
endif()

message(STATUS "Verbose log:              ${VERBOSE_LOG}")
if(VERBOSE_LOG)
    target_compile_definitions(core PUBLIC VERBOSE_LOG)
endif()

if(BUILD_EXECUTOR OR BUILD_JAVA_API OR BUILD_SERVER)
    message(STATUS "Building listeners..")
    target_compile_definitions(core PUBLIC BUILD_LISTENERS)
    target_compile_definitions(solver PUBLIC BUILD_LISTENERS)
endif()

message(STATUS "Optimize for native arch: ${OPTIMIZE_FOR_NATIVE_ARCH}")
if(OPTIMIZE_FOR_NATIVE_ARCH)
    if(MSVC)
        message(STATUS "Using AVX2..")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -arch:AVX2")
    else()
        message(STATUS "Using native optimization..")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
    endif()
endif()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
