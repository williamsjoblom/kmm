#include "kmm_drivers/MotorDriver.hpp"

namespace kmm_drivers {

MotorDriver::MotorDriver(ros::NodeHandle* nh) : nh_(nh) {

  sub_ = nh_->subscribe("cmd_vel", 1, &MotorDriver::cmd_vel_cb, this);

}

void MotorDriver::cmd_vel_cb(geometry_msgs::Twist msg) {

  ROS_INFO("x: %.3f, y: %.3f, z: %.3f", msg.linear.x, msg.linear.y, msg.angular.z);

}

}
