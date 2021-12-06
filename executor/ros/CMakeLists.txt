cmake_minimum_required(VERSION 3.0.2)
project(deliberative_tier)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)
include(GenerateExportHeader)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

## Declare ROS messages
add_message_files(
    FILES
    deliberative_state.msg
    flaw_created.msg
    flaw_state_changed.msg
    flaw_cost_changed.msg
    flaw_position_changed.msg
    current_flaw.msg
    resolver_created.msg
    resolver_state_changed.msg
    current_resolver.msg
    causal_link_added.msg
    timelines.msg
    time.msg
)

## Declare ROS services
add_service_files(
    FILES
    can_start.srv
    start_task.srv
    end_task.srv
    task_finished.srv
    create_reasoner.srv
    destroy_reasoner.srv
    new_requirement.srv
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

add_subdirectory(../../smt ${CMAKE_INSTALL_BINDIR}/smt)
add_subdirectory(../../smt/json ${CMAKE_INSTALL_BINDIR}/smt/json)
add_subdirectory(../../riddle ${CMAKE_INSTALL_BINDIR}/riddle)
add_subdirectory(../../core ${CMAKE_INSTALL_BINDIR}/core)
add_subdirectory(../../solver ${CMAKE_INSTALL_BINDIR}/solver)
add_subdirectory(../../executor ${CMAKE_INSTALL_BINDIR}/executor)
target_compile_definitions(core PUBLIC BUILD_LISTENERS)
target_compile_definitions(solver PUBLIC BUILD_LISTENERS BUILD_EXECUTOR)

add_executable(${PROJECT_NAME} main.cpp deliberative_manager.cpp deliberative_executor.cpp deliberative_solver_listener.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include ${catkin_INCLUDE_DIRS} solver executor)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} json smt core riddle solver executor)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})