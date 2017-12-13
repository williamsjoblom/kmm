#include "kmm_exploration/Exploration.hpp"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>
#include <float.h>

namespace kmm_exploration{

  Exploration::Exploration(ros::NodeHandle nh)
  : nh_(nh), navigation_client_("navigation", true),
  remove_walls_client_("remove_walls", true)
  {
    // Subscribers
    end_points_sub_ = nh_.subscribe("end_points", 1, &Exploration::end_points_callback, this);
    path_sub_ = nh_.subscribe("path", 1, &Exploration::path_callback, this);
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

    has_target_end_point_ = false;

    target_unreachable_cnt_ = 0;

    returning_ = false;
    finished_mapping_ = false;

    // Services
    auto_mode_service_ = nh_.advertiseService("set_auto_mode", &Exploration::set_auto_mode, this);

    // ROS parameters
    if (!nh_.getParam("/map_rows", map_rows_)) {
        ROS_ERROR("Couldn't set map_rows_ in Exploration!");
        assert(false);
    }

    if (!nh_.getParam("/map_cols", map_cols_)) {
        ROS_ERROR("Couldn't set map_cols_ in Exploration!");
        assert(false);
    }

    if (!nh_.getParam("/cell_size", cell_size_)) {
        ROS_ERROR("Couldn't set cell_size_ in Exploration!");
        assert(false);
    }
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

  bool Exploration::is_target_unreachable() {
    bool found_target_unreachable = has_target_end_point_ && path_.empty();
    if (found_target_unreachable) {
      target_unreachable_cnt_++;
      if (target_unreachable_cnt_ >= 10) {
        return true;
      }
    }
    return false;
  }

  bool Exploration::is_target_unexplorable() {
    return has_target_end_point_ && is_at_target_position();
  }

  Eigen::Vector2f Exploration::get_new_target(Eigen::Vector2f closest_end_point) {

    float new_target_x = closest_end_point.x() + (closest_end_point.x() - pos_x_ > 0 ? 0.2 : - 0.2);
    float new_target_y = closest_end_point.y() + (closest_end_point.y() - pos_y_ > 0 ? 0.2 : - 0.2);

    float x_lower_limit = 0;
    float x_upper_limit = cell_size_ * map_rows_;

    float abs_y_limit = cell_size_ * (map_cols_ - 1) / 2; // cell_size * (w_ - 1) / 2
    float y_lower_limit = -abs_y_limit;
    float y_upper_limit = abs_y_limit;

    // Check if out of map bounds, in that case adjust target
    if (new_target_x < x_lower_limit) {
      new_target_x += cell_size_;
    } else if (new_target_x > x_upper_limit) {
      new_target_x -= cell_size_;
    }

    if (new_target_y < y_lower_limit) {
      new_target_y += cell_size_;
    } else if (new_target_y > y_upper_limit) {
      new_target_y -= cell_size_;
    }

    Eigen::Vector2f new_target(new_target_x, new_target_y);

    return new_target;
  }

  /*
   * Callback for end_points. If robot is not in manual mode, eventually updates
   * target and publishes and sets goal based on if previous target is explored.
   */
  void Exploration::end_points_callback(sensor_msgs::PointCloud msg) {
    if (auto_mode_) {
      Eigen::Vector2f closest_end_point;

      bool are_end_points = msg.points.size();
      float min_distance = FLT_MAX;

      for (geometry_msgs::Point32 end_point : msg.points) {
        float distance = std::sqrt(std::pow(end_point.x - pos_x_, 2)
          + std::pow(end_point.y - pos_y_ , 2));

        bool point_equals_prev_target = end_point.x == target_end_point_.x()
          && end_point.y == target_end_point_.y();
        bool check_on_old_target = point_equals_prev_target && !was_in_manual_mode_;
        if (check_on_old_target) {

          if (is_target_unreachable() || is_target_unexplorable()) {
            // Found illegal wall, remove it and keep looking for new target
            send_remove_walls();
          } else {
            // Keep old end point as target
            return;
          }

        } else if (distance < min_distance) {
          Eigen::Vector2f new_closest_end_point(end_point.x, end_point.y);
          closest_end_point = new_closest_end_point;
          min_distance = distance;
        }
      }

      bool done_exploring = !are_end_points && is_at_start_position();
      if (!done_exploring) {

        Eigen::Vector2f new_target;

        if (are_end_points) {
          target_end_point_ = closest_end_point;
          new_target = get_new_target(closest_end_point);
          has_target_end_point_ = true;
          returning_ = false;

        } else {
          Eigen::Vector2f start_position(0.2, 0.2);
          new_target = start_position;
          has_target_end_point_ = false;
          returning_ = true;
        }

        bool is_new_target = new_target.x() != target_x_
          || new_target.y() != target_y_;

        bool target_not_set = is_new_target || was_in_manual_mode_;

        if (target_not_set) {
          set_new_target(new_target);
          was_in_manual_mode_ = false;
        }
      }

    } else { // Force target update when entering auto-mode
      was_in_manual_mode_ = true;
      returning_ = false;
    }
  }

/*
  Checks if value is new and in that case publishes and sends new goal.
*/
  void Exploration::set_new_target(Eigen::Vector2f new_target) {
    target_unreachable_cnt_ = 0;
    target_x_ = new_target.x();
    target_y_ = new_target.y();
    send_goal();
  }

/*
  Sends a new travelling goal to action server.
*/
  void Exploration::send_goal() {
    kmm_navigation::MoveToGoal goal;
    goal.x = target_x_;
    goal.y = target_y_;
    goal.angle = 0;
    navigation_client_.sendGoal(goal); // Send new goal
  }

  /*
   *  Sends an end point to mapping to remove it and the walls surrounding it.
   */
  void Exploration::send_remove_walls() {
    kmm_mapping::RemoveWallsGoal end_point;
    end_point.x = target_end_point_.x();
    end_point.y = target_end_point_.y();
    remove_walls_client_.sendGoal(end_point);
    if (remove_walls_client_.waitForResult()) {
      has_target_end_point_ = false;
    } else {
      ROS_ERROR("Failed to remove illegal end point and its walls!");
      assert(false);
    }
  }

  void Exploration::position_callback(geometry_msgs::PoseWithCovarianceStamped msg) {
    pos_x_ = msg.pose.pose.position.x;
    pos_y_ = msg.pose.pose.position.y;
    angle_ = msg.pose.pose.orientation.z;
    if (returning_ && is_at_start_position()) {
      finished_mapping_ = true;
      returning_ = false;
    }
  }

  void Exploration::path_callback(geometry_msgs::PoseArray msg) {
    path_.clear();
    for (const auto &p : msg.poses) {
      Eigen::Vector2f point(p.position.x, p.position.y);
      path_.push_back(point);
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

  bool Exploration::is_at_target_position() {
    float eps = 0.05;
    float diff_x = fabs(pos_x_ - target_x_);
    float diff_y = fabs(pos_y_ - target_y_);
    bool is_at_target_position = (diff_x < eps) && (diff_y < eps);
    return is_at_target_position;
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
    if (finished_mapping_) { // Only send finished mapping once
      finished_mapping_ = false;
    }
  }
}
