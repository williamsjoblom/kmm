#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int8MultiArray.h"
#include "kmm_mapping/wall_positions.h"
#include <Eigen/Dense>
#include <iostream>
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
  void increment_wall_point_count(std::vector<WallPointCount>& wall_point_counts,
    bool horizontal, int row, int col);
  void update_end_points(int row, int col, bool horizontal);

private:
  ros::NodeHandle nh_;
  // Subscribers
  ros::Subscriber mapping_sub_;
  // Publishers
  ros::Publisher mapping_wall_pos_pub_;
  ros::Publisher mapping_wall_arr_pub_;
  ros::Publisher mapping_end_points_pub_;

  // Message variables
  kmm_mapping::wall_positions wall_positions_msg_;
  std_msgs::Int8MultiArray wall_arr_msg_;
  sensor_msgs::PointCloud end_points_msg_;
  int msg_cnt_;

  // Wall point counts
  std::vector<WallPointCount> hor_wall_point_counts_;
  std::vector<WallPointCount> ver_wall_point_counts_;
  int pnt_cnt_req_ = 7;
  int times_req_ = 5;

  // Wall array
  int w_; // Grid width
  int offset_; // Used for wall_arr index calcs

  // End points
  std::vector<Eigen::Vector2f> end_points_;
};

}
