cmake_minimum_required(VERSION 2.8.3)
project(mirrax_simple_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)