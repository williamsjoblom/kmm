#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <kmm_navigation/MoveToAction.h>
#include <kmm_navigation/MoveToGoal.h>
#include <actionlib/client/simple_action_client.h>

namespace kmm_exploration {

class Target {
public:
  Target(ros::NodeHandle nh);
  ~Target();

  void end_points_callback(sensor_msgs::PointCloud msg);
  void position_callback(geometry_msgs::PoseWithCovarianceStamped msg);
  void publish_target();
  void send_goal();
  void update_target(float new_x, float new_y);

private:
  ros::NodeHandle nh_;
  //Stores the returned target position
  float x_;
  float y_;
  //Stores robot coords
  float pos_x_;
  float pos_y_;
  float angle_;
  //Stores last point chosen
  geometry_msgs::Point32 target_;

  //Action Client
  actionlib::SimpleActionClient<kmm_navigation::MoveToAction> navigation_client_;

  // Subscribers
  ros::Subscriber position_sub_;
  ros::Subscriber end_points_sub_;

  // Publishers
  ros::Publisher target_pub_;
};

}