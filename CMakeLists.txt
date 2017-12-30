cmake_minimum_required(VERSION 3.1)

project ( oRatio VERSION 0.2 LANGUAGES CXX )

set ( CMAKE_CXX_STANDARD 14 )

file( GLOB SOURCES main.cpp smt-lib/*.cpp smt-lib/lra/*.cpp smt-lib/ov/*.cpp core-lib/*.cpp core-lib/parser/*.cpp core-lib/parser/declarations/*.cpp core-lib/parser/statements/*.cpp core-lib/parser/expressions/*.cpp solver/*.cpp solver/types/*.cpp )
if( CMAKE_BUILD_TYPE MATCHES Debug )
  list( APPEND SOURCES solver/listener/solver_listener.cpp solver/listener/socket_listener.cpp ) 
endif()

include_directories( smt-lib smt-lib/lra smt-lib/ov core-lib core-lib/parser core-lib/parser/declarations core-lib/parser/statements core-lib/parser/expressions solver solver/listener solver/types )

configure_file(solver/init.rddl ${CMAKE_BINARY_DIR}/init.rddl COPYONLY)

add_executable( ${PROJECT_NAME} ${SOURCES} )

if( CMAKE_BUILD_TYPE MATCHES Debug )
  if(WIN32)
    target_link_libraries( ${PROJECT_NAME} wsock32 ws2_32 #[[-PROFILE]] )
  endif()
  include( CTest )
  add_subdirectory( smt-lib/tests )
  add_subdirectory( core-lib/tests )
endif()