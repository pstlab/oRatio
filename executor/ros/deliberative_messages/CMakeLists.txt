cmake_minimum_required(VERSION 3.0.2)
project(deliberative_messages)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

## Declare ROS messages
add_message_files(
    FILES
    deliberative_state.msg
    timelines.msg
    time.msg
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()