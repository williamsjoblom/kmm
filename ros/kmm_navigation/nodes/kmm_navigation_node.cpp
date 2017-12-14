#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include "kmm_navigation/Navigation.hpp"

/*
 * The kmm_navigation_node is responsible for navigating to a target position.
 * This includes generating a path and following it.
 */

int main(int argc, char **argv) {
  ros::init(argc, argv, "kmm_navigation");
  ros::NodeHandle nh;
  kmm_navigation::Navigation n(nh);
  ros::spin();
}
