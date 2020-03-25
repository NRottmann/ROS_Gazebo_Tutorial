cmake_minimum_required(VERSION 2.8.3)
project(pal_person_detector_opencv)

find_package(catkin REQUIRED COMPONENTS image_transport
                                        message_generation
					geometry_msgs
                                        cv_bridge
                                        roscpp)

find_package(OpenCV REQUIRED)

add_message_files(
	DIRECTORY msg
	FILES
		Detection2d.msg
		Detections2d.msg
)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime image_transport cv_bridge roscpp
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(pal_person_detector_opencv src/person_detector.cpp)
target_link_libraries(pal_person_detector_opencv ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(pal_person_detector_opencv ${catkin_EXPORTED_TARGETS} pal_person_detector_opencv_generate_messages_cpp)

install(TARGETS pal_person_detector_opencv
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
