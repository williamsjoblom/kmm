#include "kmm_navigation/Map.hpp"
#include "ros/ros.h"

namespace kmm_navigation {

  Map::Map(int map_rows, int map_cols, float cell_size)
  : h_(map_rows),
    w_(map_cols),
    cs_(cell_size) {
    offset_ = (w_ - 1) / 2;
    walls_size_ = (w_ + (w_ + 1)) * h_ + w_;

    // Initialize wall vector with 0's.
		for (int i = 0; i < walls_size_; i++) {
			walls_.push_back(0);
    };
  }

  Map::~Map() {
  }

  int Map::get_rows() {
    return h_;
  }

  int Map::get_cols() {
    return w_;
  }

  int Map::get_offset() {
    return offset_;
  }

  float Map::get_cell_size() {
    return cs_;
  }

  Eigen::Vector2f Map::get_cell(Eigen::Vector2f grid_pos) {
    ROS_INFO_STREAM("cs_ " << cs_);
    float x = grid_pos.x();
    float y = grid_pos.y();
    float cell_x = ((x - (cs_/2)) - remainder(x - (cs_/2), cs_)) / cs_;
    float cell_y = ((y - (cs_/2)) - remainder(y - (cs_/2), cs_)) / cs_;
    Eigen::Vector2f cell_pos;
    cell_pos[0] = cell_x;
    cell_pos[1] = cell_y;
    return cell_pos;
  }

  bool Map::is_wall_north_of_cell(Eigen::Vector2f cell) {
    int x = round(cell.x());
    int y = round(cell.y());
    int index = (x + 1)*(w_ + 1) + (x + 1)*w_ + offset_ + y;
    if (index >= walls_size_) {
      std::cout << "Function is_wall_above_cell() called with invalid cell!\n";
      return true;
    }
    return Map::walls_[index];
  }

  bool Map::is_wall_south_of_cell(Eigen::Vector2f cell) {
    int x = round(cell.x());
    int y = round(cell.y());
    int index = x*(w_ + 1) + x*w_ + offset_ + y;
    if (index >= walls_size_) {
      std::cout << "Function is_wall_below_cell() called with invalid cell!\n";
      return true;
    }
    return walls_[index];
  }

  bool Map::is_wall_east_of_cell(Eigen::Vector2f cell) {
    int x = round(cell.x());
    int y = round(cell.y());
    int index = x*(w_ + 1) + (x + 1)*w_ + offset_ + y;
    if (index >= walls_size_) {
      std::cout << "Function is_wall_right_cell() called with invalid cell!\n";
      return true;
    }
    return walls_[index];
  }

  bool Map::is_wall_west_of_cell(Eigen::Vector2f cell) {
    int x = round(cell.x());
    int y = round(cell.y());
    int index = x*(w_ + 1) + (x + 1)*w_ + offset_ + y + 1;
    if (index >= walls_size_) {
      std::cout << "Function is_wall_left_cell() called with invalid cell!\n";
      return true;
    }
    return walls_[index];
  }

  bool Map::is_north_reachable_from_cell(Eigen::Vector2f cell) {
    return !is_wall_north_of_cell(cell) && cell.x() < (h_ - 1);
  }

  bool Map::is_south_reachable_from_cell(Eigen::Vector2f cell) {
    return !is_wall_south_of_cell(cell) && cell.x() > 0;
  }

  bool Map::is_west_reachable_from_cell(Eigen::Vector2f cell) {
    return !is_wall_west_of_cell(cell) && cell.y() < offset_;
  }

  bool Map::is_east_reachable_from_cell(Eigen::Vector2f cell) {
    return !is_wall_east_of_cell(cell) && cell.y() > -offset_;
  }
}
