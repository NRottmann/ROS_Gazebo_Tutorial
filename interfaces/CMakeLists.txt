cmake_minimum_required(VERSION 2.8.3)
project(interfaces)

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_C_COMPILER "gcc")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  Odometry.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
    geometry_msgs
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${src}
)
