#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <kmm_navigation/MoveToAction.h>
#include <kmm_navigation/MoveToGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <kmm_mapping/RemoveWallsAction.h>
#include <kmm_mapping/RemoveWallsGoal.h>
#include <std_srvs/SetBool.h>
#include <vector>

namespace kmm_exploration {

class Exploration {
public:
  Exploration(ros::NodeHandle nh);
  ~Exploration();

private:
  ros::NodeHandle nh_;

  // Map info
  int map_rows_;
  int map_cols_;
  float cell_size_;

  // Bool for manual or autonomous mode
  bool auto_mode_;
  bool was_in_manual_mode_;

  // Stores robot position
  float pos_x_;
  float pos_y_;
  float angle_;

  // Stores the current target position
  float target_x_;
  float target_y_;

  // Stores last end point chosen
  bool has_target_end_point_;
  Eigen::Vector2f target_end_point_;

  // Count for times a target has been unreachable
  int target_unreachable_cnt_;

  // Bool for saying if we are done mapping and are returning to start
  bool returning_;

  // Bool for saying if mapping is finished and we have returned to start
  bool finished_mapping_;

  // Path
  std::vector<Eigen::Vector2f> path_;

  //Action Client
  actionlib::SimpleActionClient<kmm_navigation::MoveToAction> navigation_client_;
  actionlib::SimpleActionClient<kmm_mapping::RemoveWallsAction> remove_walls_client_;

  // Subscribers
  ros::Subscriber end_points_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber position_sub_;

  // Publishers
  ros::Publisher auto_mode_pub_;
  ros::Publisher finished_mapping_pub_;

  // Timers
  ros::Timer publish_auto_mode_timer_;
  ros::Timer publish_finished_mapping_timer_;

  // Services
  ros::ServiceServer auto_mode_service_;

  bool set_auto_mode(std_srvs::SetBool::Request &req,
         std_srvs::SetBool::Response &res);

  bool is_target_unreachable();
  bool is_target_unexplorable();

  Eigen::Vector2f get_new_target(Eigen::Vector2f closest_end_point);

  void end_points_callback(sensor_msgs::PointCloud msg);
  void position_callback(geometry_msgs::PoseWithCovarianceStamped msg);
  void path_callback(geometry_msgs::PoseArray msg);

  bool is_at_start_position();
  bool is_at_target_position();

  void send_remove_walls();
  void set_new_target(Eigen::Vector2f new_target);
  void send_goal();

  void publish_auto_mode(const ros::TimerEvent&);
  void publish_finished_mapping(const ros::TimerEvent&);
};

}
