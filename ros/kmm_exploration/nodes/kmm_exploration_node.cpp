#include "ros/ros.h"
#include "kmm_exploration/Exploration.hpp"

/*
 * The exploration node selects targets to navigate to in order to explore
 * the area. It also detects illegal walls.
*/

int main(int argc, char **argv) {
  ros::init(argc, argv, "kmm_exploration");

  ros::NodeHandle nh;

  kmm_exploration::Exploration e(nh);

  ros::spin();
}
