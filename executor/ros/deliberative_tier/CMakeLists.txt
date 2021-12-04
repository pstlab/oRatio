cmake_minimum_required(VERSION 3.0.2)
project(deliberative_tier)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)
include(GenerateExportHeader)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp deliberative_messages deliberative_services)

## Declare a catkin package
catkin_package()

add_subdirectory(../../../smt ${CMAKE_INSTALL_BINDIR}/smt)
add_subdirectory(../../../smt/json ${CMAKE_INSTALL_BINDIR}/smt/json)
add_subdirectory(../../../riddle ${CMAKE_INSTALL_BINDIR}/riddle)
add_subdirectory(../../../core ${CMAKE_INSTALL_BINDIR}/core)
add_subdirectory(../../../solver ${CMAKE_INSTALL_BINDIR}/solver)
add_subdirectory(../../../executor ${CMAKE_INSTALL_BINDIR}/executor)
target_compile_definitions(core PUBLIC BUILD_LISTENERS)
target_compile_definitions(solver PUBLIC BUILD_LISTENERS BUILD_EXECUTOR)

add_executable(deliberative_tier main.cpp deliberative_manager.cpp deliberative_executor.cpp deliberative_solver_listener.cpp)
target_include_directories(deliberative_tier PRIVATE include ${catkin_INCLUDE_DIRS} solver executor)
target_link_libraries(deliberative_tier ${catkin_LIBRARIES} json smt core riddle solver executor)
add_dependencies(deliberative_tier ${catkin_EXPORTED_TARGETS})