#include "kmm_navigation/Navigation.hpp"
#include "kmm_navigation/MoveToGoal.h"

namespace kmm_navigation {

  Navigation::Navigation(ros::NodeHandle nh)
  : nh_(nh),
    action_server_(nh_, "navigation", boost::bind(&Navigation::navigation_callback, this, _1), false) {

    // Publishers
    path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("path", 1);

    // Subscribers
    walls_sub_ = nh_.subscribe("walls", 1, &Navigation::walls_callback, this);
    position_sub_ = nh_.subscribe("position", 1, &Navigation::position_callback, this);

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

    ROS_INFO("%d, %d, %f", map_rows, map_cols, cell_size);

    map_ = new Map(map_rows, map_cols, cell_size);

    path_finder_ = new PathFinder(map_);

    action_server_.start();
  }

  Navigation::~Navigation() {
    delete path_finder_;
    delete map_;
  }

  void Navigation::navigation_callback(const kmm_navigation::MoveToGoalConstPtr &goal) {
    ROS_INFO("Got new navigation request to x: %.2f, y: %.2f, angle: %.2f", goal->x, goal->y, goal->angle);

    ros::Rate rate(3); // 3 Hz

    ROS_INFO("Calculate path!");

    Eigen::Vector2f target(goal->x, goal->y);
    std::vector<Eigen::Vector2f> path = path_finder_->find_path(pos_, target);
    publish_path(path);

    while (true) {
      ROS_INFO("Drive along path!");

      if (action_server_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("Preemted!");
        action_server_.setPreempted();
        return;
      }

      rate.sleep();
    }

    ROS_INFO("Roboten åkte hela vägen fram till target!");
    action_server_.setSucceeded(result_);
  }

  void Navigation::walls_callback(std_msgs::Int8MultiArray msg) {
    for (int i = 0; i < map_->walls_size_; i++) {
      map_->walls_[i] = msg.data[i];
    };
    return;
  }

  void Navigation::position_callback(geometry_msgs::PoseWithCovarianceStamped msg) {
    pos_[0] = msg.pose.pose.position.x;
    pos_[1] = msg.pose.pose.position.y;
  }

  void Navigation::publish_path(std::vector<Eigen::Vector2f> path) {
    ROS_INFO("Path size() = %d", path.size());
    geometry_msgs::PoseArray path_msg;
    for (Eigen::Vector2f& p : path) {
      geometry_msgs::Pose pose;
      pose.position.x = p.x();
      pose.position.y = p.y();
      path_msg.poses.push_back(pose);
      ROS_INFO("x: %f, y: %f", p.x(), p.y());
    }
    path_pub_.publish(path_msg);
  }
}
