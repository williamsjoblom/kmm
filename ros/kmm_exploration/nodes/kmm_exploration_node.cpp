#include "ros/ros.h"
#include "kmm_exploration/Exploration.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "kmm_exploration");

  ros::NodeHandle nh;

  kmm_exploration::Exploration e(nh);

  ros::spin();
}
