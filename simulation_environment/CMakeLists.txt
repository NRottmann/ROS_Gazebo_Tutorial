cmake_minimum_required(VERSION 2.8.3)
project(simulation_environment)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_ros
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  interfaces
)

# Depending on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs interfaces message_runtime
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${src}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(odometry_plugin SHARED src/odometry_plugin.cpp)
target_link_libraries(odometry_plugin ${GAZEBO_LIBRARIES})

# Gazebo set flags
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

