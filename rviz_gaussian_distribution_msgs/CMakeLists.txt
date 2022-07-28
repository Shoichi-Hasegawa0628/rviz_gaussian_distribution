cmake_minimum_required(VERSION 3.0.2)

project(rviz_gaussian_distribution_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
    message_generation
)

add_message_files(
  FILES
    GaussianDistribution.msg
)

add_action_files(
  FILES
    Add.action
    Remove.action
    Clear.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    message_runtime
    std_msgs
)
