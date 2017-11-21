#include "kmm_navigation/Map.hpp"

namespace kmm_navigation {

  Map::Map(int grid_width)
  : w_(grid_width) {
    offset_ = (w_ - 1) / 2;
    // Initialize wall array with 0's.
		for (int i = 0; i < 1500; i++) { // 53x103 = (2*height + 1) x (2*width + 1)
			wall_arr_[i] = 0;
    };
  }

  Map::~Map() {
  }

  Eigen::Vector2f Map::get_cell(Eigen::Vector2f grid_pos) {
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
    int x = round(cell.x());
    int y = round(cell.y());
    int index = (x + 1)*(w_ + 1) + (x + 1)*w_ + offset_ + y;
    if (index >= 1500) {
      std::cout << "Function is_wall_above_cell() called with invalid cell!\n";
      return true;
    }
    return Map::wall_arr_[index];
  }

  bool Map::is_wall_below_cell(Eigen::Vector2f cell) {
    int x = round(cell.x());
    int y = round(cell.y());
    int index = x*(w_ + 1) + x*w_ + offset_ + y;
    if (index >= 1500) {
      std::cout << "Function is_wall_below_cell() called with invalid cell!\n";
      return true;
    }
    return wall_arr_[index];
  }

  bool Map::is_wall_right_cell(Eigen::Vector2f cell) {
    int x = round(cell.x());
    int y = round(cell.y());
    int index = x*(w_ + 1) + (x + 1)*w_ + offset_ + y;
    if (index >= 1500) {
      std::cout << "Function is_wall_right_cell() called with invalid cell!\n";
      return true;
    }
    return wall_arr_[index];
  }

  bool Map::is_wall_left_cell(Eigen::Vector2f cell) {
    int x = round(cell.x());
    int y = round(cell.y());
    int index = x*(w_ + 1) + (x + 1)*w_ + offset_ + y + 1;
    if (index >= 1500) {
      std::cout << "Function is_wall_left_cell() called with invalid cell!\n";
      return true;
    }
    return wall_arr_[index];
  }
}
