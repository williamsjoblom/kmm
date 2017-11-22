#include "ros/ros.h"
#include "kmm_navigation/Smooth.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "kmm_smooth");
  ros::NodeHandle nh;
  kmm_navigation::Smooth s(nh);
  ros::spin();
}
