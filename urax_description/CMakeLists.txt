cmake_minimum_required(VERSION 2.8.3)
project(urax_description)

find_package(catkin REQUIRED COMPONENTS
  robot_state_publisher
  urdf
  xacro
  geometry_msgs
  # gazebo_msgs
  # hector_gazebo_plugins
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)