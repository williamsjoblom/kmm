#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include "kmm_mapping/wall_positions.h"
#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <vector>
#include <math.h>
#include <algorithm>

namespace kmm_mapping {

/* Struct to keep track of how many collisions there has been on a wall */
struct WallPointCount {
  Eigen::Vector2f position;
  int pnt_cnt; // Number of points located on this wall in one scan
  int times; // Number of times points have suggested wall here
  bool published; // True if this wall has been published as wall
};

class Mapping {
public:

  Mapping(ros::NodeHandle nh);
  ~Mapping();

  void mapping_callback(const sensor_msgs::PointCloud::ConstPtr& msg);
  WallPointCount make_wall_point_count(int row, int col, int cnt);
  void reset_wall_point_counts();
  std::vector<WallPointCount> increment_wall_point_count(
    std::vector<WallPointCount> wall_point_counts,
    bool horizontal, int row, int col);
  int random(const int a, const int b);


private:
  ros::NodeHandle nh_;
  // Subscribers
  ros::Subscriber mapping_sub_;
  // Publishers
  ros::Publisher mapping_pub_;

  // Message variables
  kmm_mapping::wall_positions wall_positions_msg_;
  int msg_cnt_;

  // Wall point counts
  std::vector<WallPointCount> hor_wall_point_counts_;
  std::vector<WallPointCount> ver_wall_point_counts_;
  int pnt_cnt_req_ = 7;
  int times_req_ = 5;

};

}
