cmake_minimum_required(VERSION 2.8.3)
project(kmm_navigation)

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
  laser_geometry
  actionlib_msgs
  actionlib
  nav_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate actions in the 'action' folder
add_action_files(
  FILES
  MoveTo.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################

# Declare a C++ library
add_library(kmm_navigation
  src/Map.cpp
)

catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS std_msgs
#  LIBRARIES kmm_navigation
#  DEPENDS system_lib
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
add_executable(kmm_navigation_node
  nodes/kmm_navigation_node.cpp
  src/Navigation.cpp
  src/Map.cpp
  src/PathFinder.cpp
  src/PathFollower.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(kmm_navigation_node
  ${kmm_navigation_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(kmm_navigation_node
  ${catkin_LIBRARIES}
)
