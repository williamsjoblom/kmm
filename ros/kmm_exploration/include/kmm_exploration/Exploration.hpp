#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <kmm_navigation/MoveToAction.h>
#include <kmm_navigation/MoveToGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/SetBool.h>

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
  geometry_msgs::Point32 target_;

  //Stores previous end points so that  path updates when walls appears
  geometry_msgs::Point32 old_points[];

  //Action Client
  actionlib::SimpleActionClient<kmm_navigation::MoveToAction> navigation_client_;

  // Subscribers
  ros::Subscriber position_sub_;
  ros::Subscriber end_points_sub_;

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
  bool is_at_start_position();
  void send_goal();
  void update_target(float new_x, float new_y);
  void publish_auto_mode(const ros::TimerEvent&);
  void publish_finished_mapping(const ros::TimerEvent&);
  bool end_points_changed(sensor_msgs::PointCloud msg);
};

}
