cmake_minimum_required(VERSION 3.0.2)
project(sven_ros)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

# Declare catkin package
catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Build VIVE pose converter node
add_executable(vive_pose_converter_node src/vive_pose_converter_node.cpp)
target_link_libraries(vive_pose_converter_node ${catkin_LIBRARIES})

# Build jump detector node
add_executable(jump_detector_node src/jump_detector_node.cpp)
target_link_libraries(jump_detector_node ${catkin_LIBRARIES})

