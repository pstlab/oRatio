cmake_minimum_required(VERSION 3.1)

project ( oRatio VERSION 0.2 LANGUAGES CXX )

set ( CMAKE_CXX_STANDARD 14 )

file( GLOB SOURCES main.cpp smt-lib/*.cpp smt-lib/lra/*.cpp smt-lib/ov/*.cpp core-lib/*.cpp core-lib/parser/*.cpp core-lib/parser/declarations/*.cpp core-lib/parser/statements/*.cpp core-lib/parser/expressions/*.cpp solver/*.cpp solver/types/*.cpp )
include_directories( smt-lib smt-lib/lra smt-lib/ov core-lib core-lib/parser core-lib/parser/declarations core-lib/parser/statements core-lib/parser/expressions solver solver/types )

add_executable( ${PROJECT_NAME} ${SOURCES} )

IF( CMAKE_BUILD_TYPE MATCHES Debug )
  include( CTest )
  add_subdirectory( smt-lib/tests )
  add_subdirectory( core-lib/tests )
ENDIF()
