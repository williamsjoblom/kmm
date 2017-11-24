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
  void set_pnt_cnt_req(int pnt_cnt_req);
  void set_times_req(int times_req);

private:
  void mapping_callback(const sensor_msgs::PointCloud::ConstPtr& msg);
  WallPointCount make_wall_point_count(int row, int col, int cnt);
  void reset_wall_point_counts();
  void increment_wall_point_count(std::vector<WallPointCount>& wall_point_counts,
    bool horizontal, int row, int col);
  void add_wall(int row, int col, bool horizontal);
  void update_end_points(int row, int col, bool horizontal);
  void publish_wall_positions();
  void publish_wall_array();
  void publish_end_points();

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
  int pnt_cnt_req_; // Number of points on wall required to increment times count
  int times_req_; // Number of times times has to be incremented for wall to be added

  // Wall array
  std::vector<int> wall_vec_;

  // End points
  std::vector<Eigen::Vector2f> end_points_;
};
}
