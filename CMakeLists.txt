cmake_minimum_required(VERSION 3.0)

project ( oRatio VERSION 0.2 LANGUAGES CXX )

file( GLOB SOURCES main.cpp "smt-lib/*.cpp" "core-lib/*.cpp" "cg-lib/*.cpp" )
include_directories( ${CMAKE_SOURCE_DIR}/smt-lib ${CMAKE_SOURCE_DIR}/core-lib ${CMAKE_SOURCE_DIR}/cg-lib )

add_executable( ${PROJECT_NAME} ${SOURCES} )

IF( CMAKE_BUILD_TYPE MATCHES Debug )
  include( CTest )
  add_subdirectory( smt-lib/tests )
ENDIF()
