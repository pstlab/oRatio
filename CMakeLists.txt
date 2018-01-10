cmake_minimum_required(VERSION 3.1)

project ( oRatio VERSION 0.2 LANGUAGES CXX )

set ( CMAKE_CXX_STANDARD 14 )

file( GLOB SOURCES main.cpp smt-lib/*.cpp smt-lib/lra/*.cpp smt-lib/ov/*.cpp core-lib/*.cpp core-lib/parser/*.cpp core-lib/parser/declarations/*.cpp core-lib/parser/statements/*.cpp core-lib/parser/expressions/*.cpp solver/*.cpp solver/types/*.cpp )
if( CMAKE_BUILD_TYPE MATCHES Debug )
  list( APPEND SOURCES solver/listener/solver_listener.cpp solver/listener/socket_listener.cpp ) 
endif()

include_directories( smt-lib smt-lib/lra smt-lib/ov core-lib core-lib/parser core-lib/parser/declarations core-lib/parser/statements core-lib/parser/expressions solver solver/listener solver/types )

configure_file(solver/init.rddl ${CMAKE_BINARY_DIR}/init.rddl COPYONLY)

option( DEFERRABLES "Use deferrables when building the planning graph" ON )
option( CHECK_UNIFICATIONS "Check unifications when building the planning graph" OFF )
option( CHECK_HYPER_FLAWS "Check hyper-flaws when building the planning graph" OFF )
option( STATISTICS "Collect statistics while building the graph and while solving the problem" ON )

message( "Use deferrables:    " ${DEFERRABLES} )
message( "Check unifications: " ${CHECK_UNIFICATIONS} )
message( "Check hyper-flaws:  " ${CHECK_HYPER_FLAWS} )
message( "Collect statistics: " ${STATISTICS} )

if( DEFERRABLES )
  add_definitions( -DDEFERRABLES )
endif()
if( CHECK_UNIFICATIONS )
  add_definitions( -DCHECK_UNIFICATIONS )
endif()
if( CHECK_HYPER_FLAWS )
  add_definitions( -DCHECK_HYPER_FLAWS )
endif()
if( STATISTICS )
  add_definitions( -DSTATISTICS )
endif()

add_executable( ${PROJECT_NAME} ${SOURCES} )

if( CMAKE_BUILD_TYPE MATCHES Debug )
  if(WIN32)
    target_link_libraries( ${PROJECT_NAME} wsock32 ws2_32 #[[-PROFILE]] )
  endif()
  include( CTest )
  add_subdirectory( smt-lib/tests )
  add_subdirectory( core-lib/tests )
  add_subdirectory( solver/tests )
endif()