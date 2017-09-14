#pragma once
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

namespace kmm_drivers {

class MotorDriver {
public:
  MotorDriver(ros::NodeHandle* nh);

  void cmd_vel_cb(geometry_msgs::Twist msg);

private:

  ros::NodeHandle* nh_;
  ros::Subscriber sub_;

};

}
