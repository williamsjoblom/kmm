#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Dense>

namespace kmm_position {

class Position {
public:
  Position(ros::NodeHandle nh);
  ~Position();

  void laser_scan_callback(sensor_msgs::LaserScan msg);
  void publish_points(std::string ns, std::vector<Eigen::Vector2f>& points, float r, float g, float b);

private:
  ros::NodeHandle nh_;
  ros::Subscriber laser_scan_sub_;
  ros::Publisher marker_pub_;
};

}
