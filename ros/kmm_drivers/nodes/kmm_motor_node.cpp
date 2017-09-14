#include "ros/ros.h"
#include "kmm_drivers/MotorDriver.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "kmm_motor");
  ros::NodeHandle nh;
  kmm_drivers::MotorDriver md(&nh);
  ROS_INFO("hej");
  ros::spin();
}
