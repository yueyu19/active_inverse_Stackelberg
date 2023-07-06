cmake_minimum_required(VERSION 3.0.2)
project(active_inverse_stackelberg_ros)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	ros_sockets_msgs
	ros_sockets
)

catkin_package(
	CATKIN_DEPENDS
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(trajectory_generator
	src/trajectory_generator.cpp
)
target_link_libraries(
	trajectory_generator
	${catkin_LIBRARIES}
)

add_executable(controller
	src/controller.cpp
)
target_link_libraries(
	controller
	X11
	${catkin_LIBRARIES}
)

install(TARGETS controller trajectory_generator
  	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)