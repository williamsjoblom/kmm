#include "ros/ros.h"
#include "kmm_navigation/Navigation.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "kmm_navigation");
  ros::NodeHandle nh;
  kmm_navigation::Navigation n(nh);
  ros::spin();
}
