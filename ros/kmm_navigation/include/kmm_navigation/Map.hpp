#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <math.h>

namespace kmm_navigation {

  class Map {
  public:

    Map(int map_rows, int map_cols, float cell_size);
    ~Map();

    Eigen::Vector2f get_cell(Eigen::Vector2f grid_pos);
    bool is_wall_above_cell(Eigen::Vector2f cell);
    bool is_wall_below_cell(Eigen::Vector2f cell);
    bool is_wall_right_cell(Eigen::Vector2f cell);
    bool is_wall_left_cell(Eigen::Vector2f cell);

    // Map variables
    int h_; // Map height in cells
    int w_; // Map width in cells
    float cs_; // Cell size in meters
    int offset_; // Used for wall array index calcs
    int walls_size_; // Size of walls_ 

    // Message variables
    std::vector<int> walls_;

  };
}
