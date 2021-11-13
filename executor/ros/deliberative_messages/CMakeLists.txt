cmake_minimum_required(VERSION 3.0.2)
project(deliberative_messages)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

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

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()