cmake_minimum_required(VERSION 2.8.3)
project(simulation_environment)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_ros
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

# Depending on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
   CATKIN_DEPENDS 
   roscpp 
   rospy 
   std_msgs 
   geometry_msgs 
   sensor_msgs 
   nav_msgs
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${src}
  ${GAZEBO_INCLUDE_DIRS}
)

# Gazebo Plugins
add_library(temp_plugin SHARED src/temp_plugin.cpp)
target_link_libraries(temp_plugin ${GAZEBO_LIBRARIES})
add_dependencies(temp_plugin ${catkin_EXPORTED_TARGETS})

# Gazebo set flags
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
