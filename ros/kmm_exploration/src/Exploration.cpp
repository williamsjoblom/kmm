#include "kmm_exploration/Exploration.hpp"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>
#include <float.h>

namespace kmm_exploration{

  Exploration::Exploration(ros::NodeHandle nh)
  : nh_(nh), navigation_client_("navigation", true)
  {
    // Subscribers
    end_points_sub_ = nh_.subscribe("end_points", 1, &Exploration::end_points_callback, this);
    position_sub_ = nh_.subscribe("position", 1, &Exploration::position_callback, this);

    // Publishers
    auto_mode_pub_ = nh_.advertise<std_msgs::Bool>("auto_mode", 1);
    finished_mapping_pub_ = nh_.advertise<std_msgs::Bool>("finished_mapping", 1);

    // Timers
    publish_auto_mode_timer_ = nh_.createTimer(ros::Duration(1. / 5), &Exploration::publish_auto_mode, this);
    publish_finished_mapping_timer_ = nh_.createTimer(ros::Duration(1. / 5), &Exploration::publish_finished_mapping, this);

    // Set initial values
    auto_mode_ = false;
    was_in_manual_mode_ = true;
    returning_ = false;
    finished_mapping_ = false;

    // Services
    auto_mode_service_ = nh_.advertiseService("set_auto_mode", &Exploration::set_auto_mode, this);
  }

  Exploration::~Exploration(){}

  /*
   * Callback for service requests to set auto mode.
   * If auto mode changes, cancels all navigation goals of navigation_client_.
   */
  bool Exploration::set_auto_mode(std_srvs::SetBool::Request &req,
         std_srvs::SetBool::Response &res) {
    bool was_in_auto_mode = auto_mode_;
    auto_mode_ = req.data;
    bool auto_mode_turned_off = was_in_auto_mode && !auto_mode_;
    bool auto_mode_turned_on = !was_in_auto_mode && auto_mode_;
    bool auto_mode_changed = auto_mode_turned_on || auto_mode_turned_off;
    if (auto_mode_changed) {
      navigation_client_.cancelAllGoals();
    }
    return true;
  }

  /*
   * Callback for end_points. If robot is not in manual mode, eventually updates
   * target and publishes and sets goal based on if previous target is explored.
   */
  void Exploration::end_points_callback(sensor_msgs::PointCloud msg) {
    if (auto_mode_) {
      geometry_msgs::Point32 closest;
      bool are_no_end_points = true;
      float min_distance = FLT_MAX;
      for (geometry_msgs::Point32 point : msg.points){
        are_no_end_points = false;
        float distance = std::sqrt(std::pow(point.x - pos_x_, 2) + std::pow(point.y - pos_y_ , 2));
        /* If point is equal to the previous, there shouldn't be a new target
         * unless we were in manual mode since last target was sent */
         bool point_equals_prev_target = point.x == target_.x && point.y == target_.y;
        if (!was_in_manual_mode_ && point_equals_prev_target) {
          return;
          //Uncomment these to make target change goal around point while moving.
          //closest = point;
          //break;
        }
        else if (distance < min_distance) {
          closest = point;
          min_distance = distance;
        }
      }
      float new_x;
      float new_y;
      if (are_no_end_points) { // Return to start position if not end points remain
        new_x = 0.2;
        new_y = 0.2;
        returning_ = true;
      } else {
        target_ = closest;
        new_x = closest.x + (closest.x - pos_x_ > 0 ? 0.2 : - 0.2);
        new_y = closest.y + (closest.y - pos_y_ > 0 ? 0.2 : - 0.2);
        returning_ = false;
      }
      update_target(new_x, new_y);
      was_in_manual_mode_ = false;
    } else { // Force target update when entering auto-mode
      was_in_manual_mode_ = true;
      returning_ = false;
    }
  }

/*
  Checks if value is new and in that case publishes and sends new goal.
*/
  void Exploration::update_target(float new_x, float new_y) {
    if (was_in_manual_mode_ || !(new_x == x_ && new_y == y_)) {
      x_ = new_x;
      y_ = new_y;
      send_goal();
    }
  }

/*
  Sends a new travelling goal to action server.
*/
  void Exploration::send_goal() {
    kmm_navigation::MoveToGoal goal;
    goal.x = x_;
    goal.y = y_;
    goal.angle = 0;
    navigation_client_.sendGoal(goal); // Send new goal
  }

  void Exploration::position_callback(geometry_msgs::PoseWithCovarianceStamped msg) {
    pos_x_ = msg.pose.pose.position.x;
    pos_y_ = msg.pose.pose.position.y;
    angle_ = msg.pose.pose.orientation.z;
    if (returning_ && is_at_start_position()) {
      finished_mapping_ = true;
      returning_ = false;
    } else {
      finished_mapping_ = false;
    }
  }

  bool Exploration::is_at_start_position() {
    float eps = 0.05;
    float start_pos_x = 0.2;
    float start_pos_y = 0.2;
    float diff_x = fabs(pos_x_ - start_pos_x);
    float diff_y = fabs(pos_y_ - start_pos_y);
    bool is_at_start_position = (diff_x < eps) && (diff_y < eps);
    return is_at_start_position;
  }

  void Exploration::publish_auto_mode(const ros::TimerEvent&) {
    std_msgs::Bool auto_mode;
    auto_mode.data = auto_mode_;
    auto_mode_pub_.publish(auto_mode);
  }

  void Exploration::publish_finished_mapping(const ros::TimerEvent&) {
    std_msgs::Bool finished_mapping;
    finished_mapping.data = finished_mapping_;
    finished_mapping_pub_.publish(finished_mapping);
  }
}
