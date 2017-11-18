#include "kmm_mapping/Map.hpp"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int8MultiArray.h"

namespace kmm_mapping {

  Map::Map(ros::NodeHandle nh)
  : nh_(nh)
  {
    // Subscribers
    map_sub_ = nh_.subscribe("/wall_arr", 1, &Map::map_callback, this);

    w_ = 51; // Grid width
    offset_ = (w_ - 1) / 2;
  }

  Mapping::~Mapping() {
  }

  void Map::map_callback(const std_msgs::Int8MultiArray::ConstPtr& msg) {
    ROS_INFO("Map recieved a message from /wall_arr");
    wall_arr_.clear();
    for (const auto p : msg->data) {
      wall_arr_.push_back(p);
    };
  }

  static Eigen::Vector2f Map::get_cell(Eigen::Vector2f grid_pos) {
    float x = grid_pos.x();
    float y = grid_pos.y();
    float cell_x = ((x - 0.2) - remainder(x - 0.2, 0.4)) / 0.4;
    float cell_y = ((y - 0.2) - remainder(y - 0.2, 0.4)) / 0.4;
    Eigen::Vector2f cell_pos;
    cell_pos[0] = cell_x;
    cell_pos[1] = cell_y;
    return cell_pos;
  }

  bool Map::is_wall_above_cell(Eigen::Vector2f cell) {
    int x = cell.x();
    int y = cell.y();
    return wall_arr_[(x + 1)*(w_ + 1) + (x + 1)*w_ + offset_ + y];
  }

  bool Map::is_wall_below_cell(Eigen::Vector2f cell) {
    int x = cell.x();
    int y = cell.y();
    return wall_arr_[x*(w_ + 1) + x*w_ + offset_ + y];
  }

  bool Map::is_wall_right_cell(Eigen::Vector2f cell) {
    int x = cell.x();
    int y = cell.y();
    return wall_arr_[x*(w_ + 1) + (x + 1)*w_ + offset_ + y];
  }

  bool Map::is_wall_left_pos(Eigen::Vector2f pos) {
    int x = cell.x();
    int y = cell.y();
    return wall_arr_[x*(w_ + 1) + (x + 1)*w_ + offset_ + y + 1];
  }
}
