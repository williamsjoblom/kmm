#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>
#include "kmm_navigation/Map.hpp"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int8MultiArray.h"

namespace kmm_navigation {
class Navigation {
public:
  Navigation(ros::NodeHandle nh);
  ~Navigation();

  void wall_array_callback(std_msgs::Int8MultiArray msg);
  void position_callback(geometry_msgs::PoseWithCovarianceStamped msg);

private:
  Map map_;

  ros::NodeHandle nh_;

  // Subscribers
  ros::Subscriber wall_array_sub_;
  ros::Subscriber position_sub_;
};
}
