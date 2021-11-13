cmake_minimum_required(VERSION 3.0.2)
project(deliberative_services)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

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