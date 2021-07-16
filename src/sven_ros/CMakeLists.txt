cmake_minimum_required(VERSION 3.0.2)
project(sven_ros)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)

## Generate added messages and services
add_message_files(DIRECTORY msg
	FILES BoolStamped.msg
)
generate_messages(DEPENDENCIES std_msgs)

# Declare catkin package
catkin_package(CATKIN_DEPENDS 
  message_runtime
)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Build VIVE pose converter node
add_executable(vive_pose_converter_node src/vive_pose_converter_node.cpp)
target_link_libraries(vive_pose_converter_node ${catkin_LIBRARIES})

# Build jump detector node
add_executable(jump_detector_node src/jump_detector_node.cpp)
target_link_libraries(jump_detector_node ${catkin_LIBRARIES})
add_dependencies(jump_detector_node sven_ros_generate_messages_cpp)

# Build sven ros controller node
catkin_install_python(PROGRAMS scripts/sven_ros_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

