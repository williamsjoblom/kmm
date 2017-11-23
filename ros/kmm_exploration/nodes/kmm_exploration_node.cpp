#include "ros/ros.h"
#include "kmm_exploration/Target.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "kmm_exploration");
  ros::NodeHandle nh;
  kmm_exploration::Target t(nh);
  ros::spin();
}