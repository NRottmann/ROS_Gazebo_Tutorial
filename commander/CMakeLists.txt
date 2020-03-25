cmake_minimum_required(VERSION 2.8.3)
project(commander)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

find_package(Eigen3 REQUIRED)

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


add_executable(example_node_kalman_filter src/example_node_kalman_filter.cpp)
target_link_libraries(example_node_kalman_filter ${catkin_LIBRARIES})
add_dependencies(example_node_kalman_filter ${catkin_EXPORTED_TARGETS})
