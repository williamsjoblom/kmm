#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Dense>
#include "kmm_position/Pose.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "kmm_position/Kalman.h"

namespace kmm_position {

class Position {
public:
  Position(ros::NodeHandle nh);
  ~Position();

  void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void cmd_vel_callback(geometry_msgs::Twist msg);
  void publish_aligned_scan(std::vector<Eigen::Vector2f>& aligned);
  void broadcast_robot_pose(const ros::TimerEvent&);
  void publish_robot_pose(const ros::TimerEvent&);
  void publish_scan_cloud(sensor_msgs::PointCloud& cloud);

private:
  ros::NodeHandle nh_;
  ros::Timer broadcast_robot_pose_timer_;
  ros::Timer publish_robot_pose_timer_;
  laser_geometry::LaserProjection projector_;

  // Subscribers
  tf::TransformListener tf_listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan>* laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan>* laser_notifier_;
  ros::Subscriber cmd_vel_sub_;

  // Publishers
  ros::Publisher aligned_scan_pub_;
  ros::Publisher position_pub_;
  ros::Publisher scan_point_cloud_pub_;
  tf::TransformBroadcaster tf_broadcaster_;

  // State estimation
  Kalman kalman_;
};

}
