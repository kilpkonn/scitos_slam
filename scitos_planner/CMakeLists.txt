cmake_minimum_required(VERSION 3.4)
project(scitos_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  visualization_msgs
  roscpp
  rospy
  tf
  scitos_common
  roslib
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  roslib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE SOURCE_FILES *.cpp)

add_library(scitos_planner ${SOURCE_FILES})

target_link_libraries(scitos_planner
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
  src/scitos_planner_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  scitos_planner
  ${OpenCV_LIBS}
)

