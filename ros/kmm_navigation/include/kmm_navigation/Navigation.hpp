#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <set>
#include <queue>
#include <math.h>
#include <algorithm>
#include <limits>
#include "kmm_navigation/Map.hpp"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int8MultiArray.h"
#include "kmm_navigation/MoveToAction.h"
#include <actionlib/server/simple_action_server.h>

namespace kmm_navigation {

// Represents cell in grid and used for path finding
struct Cell {
  double cost;
  bool visited;
  Cell* previous;
  int row;
  int col;
  bool operator<(const Cell& cell) const
  {
      return cell.cost < cost;
  }
};

class Navigation {
public:
  Navigation(ros::NodeHandle nh);
  ~Navigation();

private:
  void navigation_callback(const kmm_navigation::MoveToGoalConstPtr &goal);
  void wall_array_callback(std_msgs::Int8MultiArray msg);
  void position_callback(geometry_msgs::PoseWithCovarianceStamped msg);
  Cell* make_cell(int row, int col);
  void target_position_callback(geometry_msgs::Twist msg);
  std::vector<Eigen::Vector2f> find_path(Cell* start, Cell* end);
  std::priority_queue<Cell> get_resorted_queue(std::priority_queue<Cell> old_queue);
  void reset_cells();
  std::set<Cell*> get_neighbors(Cell* cell);
  std::vector<Eigen::Vector2f> get_path(Cell* start, Cell* end);
  std::vector<Eigen::Vector2f> make_smooth(const std::vector<Eigen::Vector2f>& path);
  void publish_path(std::vector<Eigen::Vector2f> path);

  Map map_;

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<kmm_navigation::MoveToAction> action_server_;
  kmm_navigation::MoveToFeedback feedback_;
  kmm_navigation::MoveToResult result_;

  // Subscribers
  ros::Subscriber wall_array_sub_;
  ros::Subscriber position_sub_;
  ros::Subscriber target_position_sub_;

  // Publishers
  ros::Publisher path_pub_;

  // Postition
  Eigen::Vector2f pos_;

  // Cells
  Cell* cells_[26][51]; // 26 is rows, 51 is cols

  // Path message
  geometry_msgs::PoseArray path_msg_;
};
}
