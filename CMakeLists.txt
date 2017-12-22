cmake_minimum_required(VERSION 3.1)

project ( oRatio VERSION 0.2 LANGUAGES CXX )

set ( CMAKE_CXX_STANDARD 14 )

file( GLOB SOURCES main.cpp smt-lib/*.cpp smt-lib/lra/*.cpp core-lib/*.cpp cg-lib/*.cpp )
include_directories( smt-lib smt-lib/lra core-lib cg-lib )

add_executable( ${PROJECT_NAME} ${SOURCES} )

IF( CMAKE_BUILD_TYPE MATCHES Debug )
  include( CTest )
  add_subdirectory( smt-lib/tests )
ENDIF()
