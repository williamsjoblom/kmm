#include "kmm_navigation/Navigation.hpp"
#include "kmm_navigation/MoveToGoal.h"

namespace kmm_navigation {

  Navigation::Navigation(ros::NodeHandle nh)
  : nh_(nh),
    action_server_(nh_, "navigation", boost::bind(&Navigation::navigation_callback, this, _1), false)
  {
    dynamic_reconfigure::Server<NavigationConfig>::CallbackType f;
    f = boost::bind(&Navigation::reconfigure_callback, this, _1, _2);
    server_.setCallback(f);

    // Publishers
    path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("path", 1);
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    // Subscribers
    walls_sub_ = nh_.subscribe("walls", 1, &Navigation::walls_callback, this);
    position_sub_ = nh_.subscribe("position", 1, &Navigation::position_callback, this);
    auto_mode_sub_ = nh_.subscribe("auto_mode", 1, &Navigation::auto_mode_callback, this);

    // Auto mode
    auto_mode_ = false;

    // Timers
    publish_path_timer_ = nh_.createTimer(ros::Duration(1. / 5), &Navigation::publish_path, this);

    // ROS parameters
    int map_rows;
    if (!nh_.getParam("/map_rows", map_rows)) {
        ROS_ERROR("Couldn't set map_rows!");
        assert(false);
    }

    int map_cols;
    if (!nh_.getParam("/map_cols", map_cols)) {
        ROS_ERROR("Couldn't set map_cols!");
        assert(false);
    }

    float cell_size;
    if (!nh_.getParam("/cell_size", cell_size)) {
        ROS_ERROR("Couldn't set cell_size!");
        assert(false);
    }

    if (!nh_.getParam("/produce_cmd_vel", produce_cmd_vel_)) {
        ROS_ERROR("Couldn't set produce_cmd_vel_!");
        assert(false);
    }

    // Helper classes
    map_ = new Map(map_rows, map_cols, cell_size);
    path_finder_ = new PathFinder(map_);

    // Start the action server when all other
    // instance variables are initiated.
    action_server_.start();
  }

  Navigation::~Navigation() {
    delete path_finder_;
    delete map_;
  }

  void Navigation::reconfigure_callback(NavigationConfig& config, int level) {
    /*path_follower_.set_error_p_constant(config.error_p_constant);
    path_follower_.set_max_velocity(config.max_velocity);*/
  }

  /*
    Action server callback. Gets a target position, calculates a path to follow
    and actively controls the robot velocity to keep it on track.
  */
  void Navigation::navigation_callback(const kmm_navigation::MoveToGoalConstPtr &goal) {
    ros::Rate rate(30);
    // Find a path from robot position to target for the robot to follow.
    Eigen::Vector2f target(goal->x, goal->y);
    path_ = path_finder_->find_path(robot_position_, target);

    bool initial_mode = auto_mode_;

    bool has_reached_target = false;
    while (!has_reached_target) {

      // The navigation request can be preemted by the client.
      // In that case we want to clear the path and stop the robot.
      bool auto_mode_turned_off = initial_mode && !auto_mode_;
      bool auto_mode_turned_on = !initial_mode && auto_mode_;
      bool auto_mode_changed = auto_mode_turned_on || auto_mode_turned_off;
      if (auto_mode_changed || action_server_.isPreemptRequested() || !ros::ok()) {
        action_server_.setPreempted();
        path_.clear();
        return;
      }

      // Calculate velocity to stay on the path.
      Eigen::Vector2f vel;
      path_follower_.get_velocity(path_, robot_position_, vel, has_reached_target);
      Eigen::Vector3f vel3(vel[0], vel[1], 0);
      Eigen::Transform<float, 3, Eigen::Affine> t(Eigen::AngleAxis<float>(robot_angle_ * -1, Eigen::Vector3f(0, 0, 1)));
      vel3 = t * vel3; // Rotate into robot frame.

      if (produce_cmd_vel_) {
        geometry_msgs::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x =  vel3[0];
        cmd_vel_msg.linear.y = vel3[1];
        cmd_vel_msg.angular.z = 0;
        cmd_vel_pub_.publish(cmd_vel_msg);
      }

      rate.sleep();
    }

    // The robot has reached the target destination.
    path_.clear();
    action_server_.setSucceeded(result_);
  }

  /*
    Listen to the wall array and update
    the underlying map data.
  */
  void Navigation::walls_callback(std_msgs::Int8MultiArray msg) {
    std::vector<int> walls;
    walls.resize(msg.data.size());
    for (int i = 0; i < msg.data.size(); i++) {
      walls[i] = msg.data[i];
    }
    map_->set_walls(walls);
  }

  /*
    Listen to and save robot position.
  */
  void Navigation::position_callback(geometry_msgs::PoseWithCovarianceStamped msg) {
    robot_position_[0] = msg.pose.pose.position.x;
    robot_position_[1] = msg.pose.pose.position.y;
    geometry_msgs::Quaternion q = msg.pose.pose.orientation;
    // Convert Quaternion to Euler angle.
    robot_angle_ = std::atan2(2*(q.x*q.y+q.z*q.w), 1-2*(std::pow(q.y, 2)+std::pow(q.z, 2)));
  }

  void Navigation::auto_mode_callback(std_msgs::Bool msg) {
    auto_mode_ = msg.data;
  }

  /*
    Publish the current planned path
    at a fixed time interval.
  */
  void Navigation::publish_path(const ros::TimerEvent&) {
    geometry_msgs::PoseArray msg;
    for (Eigen::Vector2f& p : path_) {
      geometry_msgs::Pose pose;
      pose.position.x = p.x();
      pose.position.y = p.y();
      msg.poses.push_back(pose);
    }
    path_pub_.publish(msg);
  }
}
