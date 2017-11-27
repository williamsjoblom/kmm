#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>
#include <vector>
#include "kmm_navigation/Map.hpp"
#include "std_msgs/Int8MultiArray.h"
#include <actionlib/server/simple_action_server.h>
#include <kmm_navigation/MoveToAction.h>
#include "PathFinder.hpp"
#include "PathFollower.hpp"

namespace kmm_navigation {

class Navigation {
public:
  Navigation(ros::NodeHandle nh);
  ~Navigation();

private:
  void navigation_callback(const kmm_navigation::MoveToGoalConstPtr &goal);
  void walls_callback(std_msgs::Int8MultiArray msg);
  void position_callback(geometry_msgs::PoseWithCovarianceStamped msg);
  void publish_path(const ros::TimerEvent&);

  Map* map_;
  PathFinder* path_finder_;
  PathFollower path_follower_;
  std::vector<Eigen::Vector2f> path_;
  Eigen::Vector2f robot_position_;
  float robot_angle_;

  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<kmm_navigation::MoveToAction> action_server_;
  kmm_navigation::MoveToFeedback feedback_;
  kmm_navigation::MoveToResult result_;

  // Subscribers
  ros::Subscriber walls_sub_;
  ros::Subscriber position_sub_;

  // Publishers
  ros::Publisher target_pub_;
  ros::Publisher path_pub_;
  ros::Publisher cmd_vel_pub_;
  ros::Timer publish_path_timer_;
};
}
