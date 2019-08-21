cmake_minimum_required(VERSION 2.8.3)
project(netft_control)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  controller_manager
  diagnostic_msgs
  diagnostic_updater
  force_torque_sensor_controller
  geometry_msgs
  hardware_interface
  roscpp
)

find_package(Boost REQUIRED COMPONENTS system thread program_options)
find_package(CURL REQUIRED)
find_package(TinyXML REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES netft_rdt_driver
  CATKIN_DEPENDS diagnostic_msgs diagnostic_updater geometry_msgs hardware_interface roscpp
)

include_directories(
 include ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(netft_rdt_driver src/netft_rdt_driver.cpp)
target_link_libraries(netft_rdt_driver 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${CURL_LIBRARIES} 
  ${TinyXML_LIBRARIES})

## Declare a cpp executable
add_executable(netft_node src/netft_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(netft_node netft_rdt_driver)

# NetFT ROS controller
add_executable(netft_ros_driver src/netft_ros_driver.cpp)
target_link_libraries(netft_ros_driver netft_rdt_driver ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS netft_rdt_driver netft_node netft_ros_driver
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
