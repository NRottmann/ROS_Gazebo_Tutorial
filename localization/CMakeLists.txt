cmake_minimum_required(VERSION 2.8.3)
project(localization)

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
   DEPENDS
     EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${src}
  ${EIGEN3_INCLUDE_DIR}
)


add_executable(kalman_filter src/kalman_filter.cpp)
target_link_libraries(kalman_filter ${catkin_LIBRARIES})
add_dependencies(kalman_filter ${catkin_EXPORTED_TARGETS})
