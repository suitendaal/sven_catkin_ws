cmake_minimum_required(VERSION 3.4)
project(franka_custom_controllers)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_hw
  franka_example_controllers
  franka_gripper
  geometry_msgs
  hardware_interface
  tf
  tf_conversions
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)

add_message_files(FILES
  ControlOptions.msg
  DemonstrationControllerState.msg
  ForceStamped.msg
  RobotState.msg
  ImpactControlState.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  franka_msgs
)

generate_dynamic_reconfigure_options(
  cfg/compliance_param.cfg
  cfg/compliance_param_demonstration.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_custom_controllers
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    eigen_conversions
    franka_hw
    franka_example_controllers
    franka_gripper
    geometry_msgs
    hardware_interface
    tf
    tf_conversions
    message_runtime
    pluginlib
    realtime_tools
    roscpp
  DEPENDS Franka
)

add_library(franka_custom_controllers
  src/impact_aware_cartesian_impedance_controller.cpp
  src/demonstration_cartesian_impedance_controller.cpp
)

add_dependencies(franka_custom_controllers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
#  ${PROJECT_NAME}_gencfg
)

target_link_libraries(franka_custom_controllers PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(franka_custom_controllers SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(franka_custom_controllers PUBLIC
  include
)

#add_executable(teleop_gripper_node
#  src/teleop_gripper_node.cpp
#)
#target_include_directories(teleop_gripper_node PUBLIC
#  ${catkin_INCLUDE_DIRS}
#)
#target_link_libraries(teleop_gripper_node PUBLIC
#  ${Franka_LIBRARIES}
#  ${catkin_LIBRARIES}
#)
#add_dependencies(teleop_gripper_node
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#  ${catkin_EXPORTED_TARGETS}
#  ${PROJECT_NAME}_generate_messages_cpp
#  ${PROJECT_NAME}_gencpp
#  ${PROJECT_NAME}_gencfg
#)

## Installation
install(TARGETS franka_custom_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES franka_custom_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#catkin_install_python(
#  PROGRAMS scripts/interactive_marker.py scripts/move_to_start.py scripts/dual_arm_interactive_marker.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(franka_custom_controllers FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_custom_controllers
    FILES ${SOURCES}
    DEPENDS franka_custom_controllers
  )
endif()
