#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace kmm_position {

class Position {
public:
  Position(ros::NodeHandle nh);
  ~Position();

  void laser_scan_callback(sensor_msgs::LaserScan msg);

private:
  ros::NodeHandle nh_;
  ros::Subscriber laser_scan_sub_;
};

}
