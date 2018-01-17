cmake_minimum_required(VERSION 3.1)

project ( oRatio VERSION 1.0.1 LANGUAGES CXX )

# Set a default build type if none was specified
set( default_build_type "Release" )
if( EXISTS "${CMAKE_SOURCE_DIR}/.git" )
  set( default_build_type "Debug" )
endif()
 
if( NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES )
  message( STATUS "Setting build type to '${default_build_type}' as none was specified." )
  set( CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE )
  # Set the possible values of build type for cmake-gui
  set_property( CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo" )
endif()

option( DEFERRABLES "Use deferrables when building the planning graph" ON )
option( CHECK_INCONSISTENCIES "Check inconsistencies at each step" OFF )
option( CHECK_UNIFICATIONS "Check unifications when building the planning graph" OFF )
option( CHECK_HYPER_FLAWS "Check hyper-flaws when building the planning graph" OFF )
option( STATISTICS "Collect statistics while building the graph and while solving the problem" ON )

message( STATUS "Use deferrables:       ${DEFERRABLES}" )
message( STATUS "Check inconsistencies: ${CHECK_INCONSISTENCIES}" )
message( STATUS "Check unifications:    ${CHECK_UNIFICATIONS}" )
message( STATUS "Check hyper-flaws:     ${CHECK_HYPER_FLAWS}" )
message( STATUS "Collect statistics:    ${STATISTICS}" )

set ( CMAKE_CXX_STANDARD 14 )

file( GLOB SOURCES main.cpp smt-lib/*.cpp smt-lib/lra/*.cpp smt-lib/ov/*.cpp core-lib/*.cpp core-lib/parser/*.cpp core-lib/parser/declarations/*.cpp core-lib/parser/statements/*.cpp core-lib/parser/expressions/*.cpp solver/*.cpp solver/types/*.cpp )
if( CMAKE_BUILD_TYPE MATCHES Debug )
  list( APPEND SOURCES solver/listener/solver_listener.cpp solver/listener/socket_listener.cpp ) 
endif()

include_directories( smt-lib smt-lib/lra smt-lib/ov core-lib core-lib/parser core-lib/parser/declarations core-lib/parser/statements core-lib/parser/expressions solver solver/listener solver/types )

configure_file( solver/init.rddl ${CMAKE_BINARY_DIR}/init.rddl COPYONLY )

if( DEFERRABLES )
  add_definitions( -DDEFERRABLES )
endif()
if( CHECK_INCONSISTENCIES )
  add_definitions( -DCHECK_INCONSISTENCIES )
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
  if( WIN32 )
    target_link_libraries( ${PROJECT_NAME} wsock32 ws2_32 #[[-PROFILE]] )
  endif()
  include( CTest )
  add_subdirectory( smt-lib/tests )
  add_subdirectory( core-lib/tests )
  add_subdirectory( solver/tests )
endif()