cmake_minimum_required(VERSION 2.8.3)
project(mirrax_gazebo)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  mirrax_description
  urax_description
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)