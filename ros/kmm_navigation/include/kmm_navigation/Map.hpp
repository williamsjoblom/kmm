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

    int get_rows();
    int get_cols();
    int get_offset();
    float get_cell_size();
    void set_walls(std::vector<int> walls);

    Eigen::Vector2f get_cell(Eigen::Vector2f grid_pos);

    bool is_wall_north_of_cell(Eigen::Vector2f cell);
    bool is_wall_east_of_cell(Eigen::Vector2f cell);
    bool is_wall_south_of_cell(Eigen::Vector2f cell);
    bool is_wall_west_of_cell(Eigen::Vector2f cell);

    bool is_north_reachable_from_cell(Eigen::Vector2f cell);
    bool is_north_east_reachable_from_cell(Eigen::Vector2f cell);
    bool is_east_reachable_from_cell(Eigen::Vector2f cell);
    bool is_south_east_reachable_from_cell(Eigen::Vector2f cell);
    bool is_south_reachable_from_cell(Eigen::Vector2f cell);
    bool is_south_west_reachable_from_cell(Eigen::Vector2f cell);
    bool is_west_reachable_from_cell(Eigen::Vector2f cell);
    bool is_north_west_reachable_from_cell(Eigen::Vector2f cell);

    bool is_north_of_cell(Eigen::Vector2f cell_1, Eigen::Vector2f cell_2);
    bool is_north_east_of_cell(Eigen::Vector2f cell_1, Eigen::Vector2f cell_2);
    bool is_east_of_cell(Eigen::Vector2f cell_1, Eigen::Vector2f cell_2);
    bool is_south_east_of_cell(Eigen::Vector2f cell_1, Eigen::Vector2f cell_2);
    bool is_south_of_cell(Eigen::Vector2f cell_1, Eigen::Vector2f cell_2);
    bool is_south_west_of_cell(Eigen::Vector2f cell_1, Eigen::Vector2f cell_2);
    bool is_west_of_cell(Eigen::Vector2f cell_1, Eigen::Vector2f cell_2);
    bool is_north_west_of_cell(Eigen::Vector2f cell_1, Eigen::Vector2f cell_2);

    bool is_wall_in_path(const std::vector<Eigen::Vector2f>& path);

  private:

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
