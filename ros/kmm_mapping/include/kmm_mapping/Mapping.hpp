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
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

namespace kmm_mapping {

/* Struct to keep track of how many collisions there has been on a wall */
struct WallPointCount {
  Eigen::Vector2f position;
  int pnt_cnt; // Number of points located on this wall in one scan
  int times; // Number of times points have suggested wall here
  bool found; // True if found after scan (enough points were on it)
  bool added; // True if this wall has been added to walls_
};

class Mapping {
public:
  Mapping(ros::NodeHandle nh);
  ~Mapping();
  void set_pnt_cnt_req(int pnt_cnt_req);
  void set_times_req(int times_req);

private:
  void calculate_wall(int row, int col, bool horizontal);
  Eigen::Vector2f get_end_point(
      int row,
      int col,
      bool horizontal,
      bool first
    );
  int count_connecting_walls_at(Eigen::Vector2f crossing);
  Eigen::Vector2f get_north_end_point(Eigen::Vector2f crossing);
  Eigen::Vector2f get_south_end_point(Eigen::Vector2f crossing);
  Eigen::Vector2f get_west_end_point(Eigen::Vector2f crossing);
  Eigen::Vector2f get_east_end_point(Eigen::Vector2f crossing);
  void remove_illegal_walls(int walls_at_1, int walls_at_2,
  Eigen::Vector2f end_point_1, Eigen::Vector2f end_point_2);
  void add_legal_wall(int wall_index, int walls_at_1, int walls_at_2,
  Eigen::Vector2f end_point_1, Eigen::Vector2f end_point_2);
  void remove_north_wall(Eigen::Vector2f crossing);
  void remove_south_wall(Eigen::Vector2f crossing);
  void remove_west_wall(Eigen::Vector2f crossing);
  void remove_east_wall(Eigen::Vector2f crossing);
  void remove_wall(int wall_index, Eigen::Vector2f end_point);
  bool wall_exists(int wall_index);
  int get_wall_index(int row, int col, int horizontal);
  bool within_bounds(int wall_index);
  void get_end_points(int row, int col, bool horizontal,
    std::vector<Eigen::Vector2f> &end_points);
  int convert_to_wall_index(Eigen::Vector2f end_point, bool horizontal);
  bool is_equal(Eigen::Vector2f vector1, Eigen::Vector2f vector2);
  void remove_end_point(Eigen::Vector2f end_point);

  void mapping_scan_callback(const sensor_msgs::PointCloud::ConstPtr& msg);
  WallPointCount make_wall_point_count(int row, int col);
  void reset_wall_point_counts();
  void increment_horizontal_wall_point_count(int row, int col);
  void increment_vertical_wall_point_count(int row, int col);
  void increment_wall_point_count(std::vector<WallPointCount>& wall_point_counts,
    bool horizontal, int row, int col);
  void add_wall_at(int row, int col, bool horizontal);
  bool is_horizontal_wall_at(int row, int col);
  bool is_vertical_wall_at(int row, int col);
  bool is_wall_at(int row, int col, bool horizontal);

  void update_end_points(
    int walls_at_1,
    int walls_at_2,
    Eigen::Vector2f end_point_1,
    Eigen::Vector2f end_point_2);
  void publish_mapping(const ros::TimerEvent&);
  void publish_walls(const ros::TimerEvent&);
  void publish_end_points(const ros::TimerEvent&);
  bool set_mapping(std_srvs::SetBool::Request &req,
         std_srvs::SetBool::Response &res);
  bool reset_map(std_srvs::SetBool::Request &req,
        std_srvs::SetBool::Response &res);

  ros::NodeHandle nh_;

  // Subscribers
  ros::Subscriber mapping_scan_sub_;

  // Publishers
  ros::Publisher mapping_pub_;
  ros::Publisher walls_pub_;
  ros::Publisher end_points_pub_;

  // Timers
  ros::Timer publish_mapping_timer_;
  ros::Timer publish_walls_timer_;
  ros::Timer publish_end_points_timer_;

  // Services
  ros::ServiceServer mapping_service_;
  ros::ServiceServer reset_map_service_;

  // Map variables
  bool mapping_; // True if mapping enabled
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
  std::vector<Eigen::Vector2f> end_points_;
};
}
