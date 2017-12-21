cmake_minimum_required(VERSION 3.0)

project ( oRatio VERSION 0.2 LANGUAGES CXX )

file( GLOB SOURCES main.cpp "smt-lib/*.cpp" "core-lib/*.cpp" "cg-lib/*.cpp" )

add_executable( ${PROJECT_NAME} ${SOURCES} )