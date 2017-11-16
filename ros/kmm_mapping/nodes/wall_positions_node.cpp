#include "ros/ros.h"
#include "kmm_mapping/Mapping.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "kmm_mapping");
  ros::NodeHandle nh;
  kmm_mapping::Mapping m(nh);
  ros::spin();
}
