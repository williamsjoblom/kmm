cmake_minimum_required(VERSION 2.8.3)
project(kmm_gamepad)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
set(CMAKE_MODULE_PATH
  /usr/share/SFML/cmake/Modules/
)

find_package(SFML 2 REQUIRED system window graphics network audio)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/Gamepad.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES kmm_gamepad
  CATKIN_DEPENDS roscpp rospy std_msgs
  #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${SFML_INCLUDE_DIR}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node
  nodes/kmm_gamepad_node.cpp
)

add_executable(kmm_gamepad_to_vel_node
  nodes/kmm_gamepad_to_vel_node.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${SFML_LIBRARIES}
)

target_link_libraries(kmm_gamepad_to_vel_node
  ${catkin_LIBRARIES}
  ${SFML_LIBRARIES}
)
