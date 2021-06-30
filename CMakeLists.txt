cmake_minimum_required(VERSION 3.0.0)
project(oRatio VERSION 2.9.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include(GenerateExportHeader)
include(GNUInstallDirs)

option(VERBOSE_LOG "Makes log more verbose" OFF)
option(PARALLELIZE "Performs some activities in parallel" OFF)

option(BUILD_EXECUTOR "Builds the executor" OFF)
option(BUILD_ROS_TOOLS "Builds the ROS tools" OFF)

option(BUILD_JAVA_API "Builds the Java API" OFF)
option(BUILD_PYTHON_API "Builds the Python API" OFF)
option(BUILD_SERVER "Builds the server" OFF)

option(OPTIMIZE_FOR_NATIVE_ARCH "Optimize for native arch" OFF)

if(BUILD_SERVER)
    set(BUILD_JAVA_API ${BUILD_SERVER})
endif()

if(BUILD_JAVA_API OR BUILD_PYTHON_API)
    set(BUILD_EXECUTOR ${BUILD_JAVA_API} OR ${BUILD_PYTHON_API})
endif()

include(CTest)
enable_testing()

add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE solver core smt smt/lra smt/ov smt/dl)
target_link_libraries(${PROJECT_NAME} PRIVATE solver)
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

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

message(STATUS "Build ROS tools:          ${BUILD_ROS_TOOLS}")
if(BUILD_ROS_TOOLS)
    add_subdirectory(executor/ros)
endif()

message(STATUS "Build Java API:           ${BUILD_JAVA_API}")
if(BUILD_JAVA_API)
    add_subdirectory(api/java/src/main/cpp)
endif()

message(STATUS "Build Python API:         ${BUILD_PYTHON_API}")
if(BUILD_PYTHON_API)
    add_subdirectory(api/python/cpp)
endif()

message(STATUS "Build server:             ${BUILD_SERVER}")
if(BUILD_SERVER)
    add_subdirectory(gui/src/main/cpp)
endif()

message(STATUS "Verbose log:              ${VERBOSE_LOG}")
if(VERBOSE_LOG)
    target_compile_definitions(core PUBLIC VERBOSE_LOG)
endif()

if(BUILD_EXECUTOR OR BUILD_JAVA_API)
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
