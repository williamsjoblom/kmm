cmake_minimum_required(VERSION 2.8.3)
project(kmm_mapping)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  message_generation
  laser_geometry
  actionlib_msgs
  actionlib
  dynamic_reconfigure
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
#add_message_files(
# FILES
# wall_positions.msg
#)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  RemoveWalls.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 geometry_msgs
 sensor_msgs
 std_msgs
 actionlib_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/Mapping.cfg
)

###################################
## catkin specific configuration ##
###################################

# Declare a C++ library
add_library(kmm_mapping
  src/Mapping.cpp
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs sensor_msgs roscpp std_msgs std_srvs message_runtime dynamic_reconfigure
  INCLUDE_DIRS include
  LIBRARIES kmm_mapping
  # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(kmm_mapping_node
  nodes/kmm_mapping_node.cpp
  src/Mapping.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(kmm_mapping_node
  ${kmm_mapping_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

## Specify libraries to link a library or executable target against
target_link_libraries(kmm_mapping_node
  kmm_mapping
  ${catkin_LIBRARIES}
)
