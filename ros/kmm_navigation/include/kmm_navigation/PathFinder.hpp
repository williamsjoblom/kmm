#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <set>
#include <queue>
#include <math.h>
#include <algorithm>
#include <limits>
#include "kmm_navigation/Map.hpp"

namespace kmm_navigation {

// Represents cell in grid and used for path finding
struct Cell {
  double cost;
  double path_cost;
  bool visited;
  Cell* previous;
  int row;
  int col;
  bool operator<(const Cell& cell) const
  {
      return cell.cost < cost;
  }
};

class PathFinder {
public:
  PathFinder(Map* map);
  ~PathFinder();
  std::vector<Eigen::Vector2f> find_path(Eigen::Vector2f from, Eigen::Vector2f to);
  std::vector<Eigen::Vector2f> make_smooth(const std::vector<Eigen::Vector2f>& path);

private:
  Cell* make_cell(int row, int col);
  std::priority_queue<Cell> get_resorted_queue(std::priority_queue<Cell> old_queue);
  void reset_cells();
  std::set<Cell*> get_neighbors(Cell* cell);
  std::vector<Eigen::Vector2f> get_path(Cell* start, Cell* end);

  Map* map_;

  // Cells
  Cell* cells_[26][51]; // 26 is rows, 51 is cols

  double diagonal_cost_;
};
}
