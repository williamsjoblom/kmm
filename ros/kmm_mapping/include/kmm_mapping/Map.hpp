#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <math.h>

namespace kmm_mapping {

  class Map {
  public:

    Map(ros::NodeHandle nh);
    ~Map();

    void mapping_callback(const std_msgs::Int8MultiArray::ConstPtr& msg);
    static Eigen::Vector2f get_cell(Eigen::Vector2f grid_pos);
    bool is_wall_above_pos(Eigen::Vector2f pos);
    bool is_wall_below_pos(Eigen::Vector2f pos);
    bool is_wall_right_pos(Eigen::Vector2f pos);
    bool is_wall_left_pos(Eigen::Vector2f pos);

  private:
    ros::NodeHandle nh_;
    // Subscribers
    ros::Subscriber map_sub_;

    // Message variables
    std::vector<int> wall_arr_;

    // Other
    int w_; // Grid width
    int offset_; // Used for wall array index calcs

  };
}
