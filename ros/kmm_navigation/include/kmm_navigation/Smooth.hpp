#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include <vector>
#include <math.h>
#include "kmm_navigation/Map.hpp"

namespace kmm_navigation {

class Smooth {
public:
  Smooth(ros::NodeHandle nh);
  ~Smooth();

  void timer_callback(const ros::TimerEvent&);
  nav_msgs::Path path_to_msg(std::vector<Eigen::Vector2f> path);
  std::vector<Eigen::Vector2f> make_smooth(const std::vector<Eigen::Vector2f>& path);

private:

  Map map_;
  ros::NodeHandle nh_;
  ros::Timer timer_;
  ros::Publisher not_smooth_pub_;
  ros::Publisher smooth_pub_;
};

}
