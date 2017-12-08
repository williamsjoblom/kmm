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

  // Bool for manual or autonomous mode
  bool auto_mode_;
  bool was_in_manual_mode_;

  // Bool for saying if we are done mapping and are returning to start
  bool returning_;

  // Bool for saying if mapping is finished and we have returned to start
  bool finished_mapping_;

  //Stores the returned target position
  float x_;
  float y_;

  //Stores robot coords
  float pos_x_;
  float pos_y_;
  float angle_;

  //Stores last point chosen
  bool has_target_end_point_;
  geometry_msgs::Point32 target_;

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
  void end_points_callback(sensor_msgs::PointCloud msg);
  void position_callback(geometry_msgs::PoseWithCovarianceStamped msg);
  void path_callback(geometry_msgs::PoseArray msg);
  bool is_at_start_position();
  bool is_at_target_position();
  void send_goal();
  void send_remove_walls();
  void update_target(float new_x, float new_y);
  void publish_auto_mode(const ros::TimerEvent&);
  void publish_finished_mapping(const ros::TimerEvent&);
};

}
