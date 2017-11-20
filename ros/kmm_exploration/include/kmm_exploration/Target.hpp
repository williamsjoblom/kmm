#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace kmm_exploration {

class Target {
public:
  Target(ros::NodeHandle nh);
  ~Target();

  void end_points_callback(sensor_msgs::PointCloud msg);
  void position_callback(geometry_msgs::PoseWithCovarianceStamped msg);
  void publish_target(float x, float y);

private:
  ros::NodeHandle nh_;
  float pos_x_;
  float pos_y_;
  float angle_;
  geometry_msgs::Point32 target_;

  // Subscribers
  ros::Subscriber position_sub_;
  ros::Subscriber end_points_sub_;

  // Publishers
  ros::Publisher target_pub_;
};

}
