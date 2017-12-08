cmake_minimum_required(VERSION 2.8.3)
project(kmm_exploration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  kmm_navigation
  kmm_mapping
  actionlib
  actionlib_msgs
  std_srvs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  actionlib_msgs
#  std_msgs
#  sensor_msgs
#  geometry_msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES kmm_exploration
  #CATKIN_DEPENDS message_runtime roscpp std_msgs geometry_msgs sensor_msgs actionlib actionlib_msgs
  #DEPENDS systemlib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${kmm_navigation_INCLUDE_DIRS}
  ${kmm_mapping_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(kmm_exploration_node
  nodes/kmm_exploration_node.cpp
  src/Exploration.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(kmm_exploration_node
  ${kmm_exploration_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  kmm_navigation # MoveTo.action
  kmm_mapping
)

## Specify libraries to link a library or executable target against
target_link_libraries(kmm_exploration_node
  kmm_navigation
  kmm_mapping
  ${catkin_LIBRARIES}
)
