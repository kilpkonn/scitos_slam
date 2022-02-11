cmake_minimum_required(VERSION 2.8.3)
project(scitos_annus_loomets_kitsing)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)