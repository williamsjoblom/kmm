#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "sensor_msgs/Imu.h"
#include <Eigen/Dense>
#include "kmm_position/Pose.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "kmm_position/Kalman.h"
#include "kmm_position/PositionConfig.h"
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

namespace kmm_position {

class Position {
public:
  Position(ros::NodeHandle nh);
  ~Position();

  void cmd_vel_callback(geometry_msgs::Twist msg);
  void imu_callback(sensor_msgs::Imu msg);
  void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void auto_mode_callback(const std_msgs::Bool::ConstPtr& msg);

  void publish_scan(ros::Publisher& pub, std::vector<Eigen::Vector2f>& scan);
  void broadcast_robot_pose(const ros::TimerEvent&);
  void publish_robot_pose(const ros::TimerEvent&);

  void reconfigure_callback(PositionConfig& config, int level);

  bool reset_position(std_srvs::SetBool::Request &req,
        std_srvs::SetBool::Response &res);

private:
  ros::NodeHandle nh_;
  ros::Timer broadcast_robot_pose_timer_;
  ros::Timer publish_robot_pose_timer_;
  laser_geometry::LaserProjection projector_;

  // Dynamic reconfigure
  dynamic_reconfigure::Server<PositionConfig> reconfigure_server_;
  PositionConfig config_;

  // Subscribers
  tf::TransformListener tf_listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan>* laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan>* laser_notifier_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber auto_mode_sub_;

  // Publishers
  ros::Publisher position_scan_pub_;
  ros::Publisher mapping_scan_pub_;
  ros::Publisher position_pub_;
  tf::TransformBroadcaster tf_broadcaster_;

  // Services
  ros::ServiceServer reset_position_service_;

  // State estimation
  Kalman kalman_;
  bool use_predictions_;
  bool use_lidar_;
  bool use_accelerometer_;
  bool use_gyroscope_;

  // Autonomous mode
  bool auto_mode_;

  // Map variables
  float cell_size_;
};

}
