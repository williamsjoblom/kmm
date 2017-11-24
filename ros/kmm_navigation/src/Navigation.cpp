#include "kmm_navigation/Navigation.hpp"
#include "kmm_navigation/MoveToGoal.h"

namespace kmm_navigation {

  Navigation::Navigation(ros::NodeHandle nh)
  : nh_(nh),
    action_server_(nh_, "navigation", boost::bind(&Navigation::navigation_callback, this, _1), false)
  {

    // Publishers
    path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("path", 1);

    // Subscribers
    walls_sub_ = nh_.subscribe("walls", 1, &Navigation::walls_callback, this);
    position_sub_ = nh_.subscribe("position", 1, &Navigation::position_callback, this);

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

  /*
    Action server callback. Gets a target position, calculates a path to follow
    and actively controls the robot velocity to keep it on track.
  */
  void Navigation::navigation_callback(const kmm_navigation::MoveToGoalConstPtr &goal) {
    ros::Rate rate(20);

    // Find a path from robot position to target for the robot to follow.
    Eigen::Vector2f target(goal->x, goal->y);
    path_ = path_finder_->find_path(robot_position_, target);

    bool has_reached_target = false;
    while (!has_reached_target) {

      // The navigation request can be preemted by the client.
      // In that case we want to clear the path and stop the robot.
      if (action_server_.isPreemptRequested() || !ros::ok()) {
        action_server_.setPreempted();
        path_.clear();
        return;
      }

      // TODO: Do the velocity control of the robot.

      rate.sleep();
    }

    // The robot has reached the target destination.
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
