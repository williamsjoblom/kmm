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

struct Crossing {
  Eigen::Vector2f position;
  int nr_of_walls // Number of walls connected at this crossing
  std::vector<int> wall_index; // The wall index, of the walls connected here.
};



class Mapping {
public:
  Mapping(ros::NodeHandle nh);
  ~Mapping();
  void set_pnt_cnt_req(int pnt_cnt_req);
  void set_times_req(int times_req);

private:
  void get_end_points(
    int row,
    int col, bool horizontal,
    std::pair<float,
    float> end_points[2]
  );
  Eigen::Vector2f Mapping::get_end_point(
    int row,
    int col,
    bool horizontal,
    bool first
  );


  void add_end_point(int row, int col, bool horizontal, bool first);
  bool within_bounds(int wall_index);
  int get_wall_index(int row, int col, int horizontal);
  int remove_crossings_from_horizontal(int row, int col, bool first);
  int remove_crossings_from_vertical(int row, int col, bool first);
  bool remove_end_point(int row, int col, bool horizontal, bool first);
  bool remove_wall(int row, int col, bool horizontal);
  int count_crossings_from_vertical(int row, int col, bool first);
  int count_crossings_from_horizontal(int row, int col, bool first);
  void check_three_way_crossing(int row, int col, bool horizontal);
  bool wall_exists(int row, int col, bool horizontal);
  void mapping_callback(const sensor_msgs::PointCloud::ConstPtr& msg);
  WallPointCount make_wall_point_count(int row, int col, int cnt);
  Crossing make_crossing(int x, int y, int count, int wall_index);
  void reset_wall_point_counts();
  void increment_wall_point_count(std::vector<WallPointCount>& wall_point_counts,
    bool horizontal, int row, int col);
  void add_wall(int row, int col, bool horizontal);
  void update_end_points(int row, int col, bool horizontal);
  void publish_walls(const ros::TimerEvent&);
  void publish_end_points(const ros::TimerEvent&);

  ros::NodeHandle nh_;

  // Subscribers
  ros::Subscriber aligned_scan_sub_;

  // Publishers
  ros::Publisher walls_pub_;
  ros::Publisher end_points_pub_;

  // Timers
  ros::Timer publish_walls_timer_;
  ros::Timer publish_end_points_timer_;

  // Map variables
  int h_; // Map rows = height
  int w_; // Map cols = width
  float cell_size_; // Size of a cell in meters
  int offset_; // Offset used to index walls_
  int walls_size_; // Size of walls_

  // Message variables
  std_msgs::Int8MultiArray walls_msg_;
  sensor_msgs::PointCloud end_points_msg_;
  int msg_cnt_;

  // Wall point counts
  std::vector<WallPointCount> hor_wall_point_counts_;
  std::vector<WallPointCount> ver_wall_point_counts_;
  int pnt_cnt_req_; // Number of points on wall required to increment times count
  int times_req_; // Number of times times has to be incremented for wall to be added

  // Wall vector
  std::vector<int> walls_;

  // End points
  std::vector<Crossing> end_points_;
};
}
