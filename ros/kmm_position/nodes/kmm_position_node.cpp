#include "ros/ros.h"
#include "kmm_position/Position.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "kmm_position");
  ros::NodeHandle nh;
  kmm_position::Position p(nh);
  ros::spin();
}
