#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <math.h>

namespace kmm_navigation {

  class Map {
  public:

    Map(int grid_width);
    ~Map();

    Eigen::Vector2f get_cell(Eigen::Vector2f grid_pos);
    bool is_wall_above_cell(Eigen::Vector2f cell);
    bool is_wall_below_cell(Eigen::Vector2f cell);
    bool is_wall_right_cell(Eigen::Vector2f cell);
    bool is_wall_left_cell(Eigen::Vector2f cell);

    // Wall array variables
    int w_; // Grid width
    int offset_; // Used for wall array index calcs

    // Message variables
    int wall_arr_[1500];

  };
}
