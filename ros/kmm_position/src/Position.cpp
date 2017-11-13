#include "kmm_position/Position.hpp"

namespace kmm_position {

  Position::Position(ros::NodeHandle nh)
  : nh_(nh)
  {
    laser_scan_sub_ = nh_.subscribe("scan", 1, &Position::laser_scan_callback, this);
  }

  Position::~Position() {

  }

  void Position::laser_scan_callback(sensor_msgs::LaserScan msg) {
    ROS_INFO("GOT SCAN!");
  }

}
