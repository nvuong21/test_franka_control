cmake_minimum_required(VERSION 2.8.3)
project(franka_controllers)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  franka_hw
  franka_msgs
  geometry_msgs
  hardware_interface
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.5.0 REQUIRED)

add_message_files(FILES
  JointTorque.msg
  Float64Array.msg
  RobotState.msg
)

generate_messages(
  DEPENDENCIES
  franka_msgs
)

generate_dynamic_reconfigure_options(
  cfg/force_controller_params.cfg
  cfg/impedance_param.cfg
  cfg/hybrid_param.cfg
)

catkin_package(
  LIBRARIES franka_controllers
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    franka_hw
    geometry_msgs
    hardware_interface
    message_runtime
    pluginlib
    realtime_tools
    roscpp
  DEPENDS Franka
)

include_directories(include
)

add_library(franka_controllers
  src/joint_position_controller.cpp
  src/joint_controller_torque_input.cpp
  src/test_controller.cpp
  src/impedance_force_controller.cpp
  src/state_controller.cpp
  src/joint_velocity_controller.cpp
  src/cartesian_velocity_controller.cpp
  src/force_controller.cpp
  src/hybrid_force_controller.cpp
)

add_dependencies(franka_controllers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
#  ${PROJECT_NAME}_gencpp
#  ${PROJECT_NAME}_gencfg
)

target_link_libraries(franka_controllers PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(franka_controllers SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(franka_controllers PUBLIC
  include
)

## Installation
install(TARGETS franka_controllers
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
install(FILES franka_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#catkin_install_python(
#  PROGRAMS scripts/interactive_marker.py scripts
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
