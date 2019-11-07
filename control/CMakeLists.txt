cmake_minimum_required(VERSION 2.8.3)
project(control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

catkin_package(
   CATKIN_DEPENDS 
   roscpp 
   rospy 
   std_msgs 
   geometry_msgs 
   sensor_msgs 
   nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${src}
)


add_executable(pdControl src/pdControl.cpp)
target_link_libraries(pdControl ${catkin_LIBRARIES})
add_dependencies(pdControl ${catkin_EXPORTED_TARGETS})
