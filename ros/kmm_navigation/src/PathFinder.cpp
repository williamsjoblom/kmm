#include "kmm_navigation/PathFinder.hpp"
#include "kmm_navigation/MoveToGoal.h"

namespace kmm_navigation {

  PathFinder::PathFinder(Map* map) : map_(map) {

    // Initiate cell array.
    for (int row = 0; row < map_->get_rows(); row++) {
      for (int col = 0; col < map_->get_cols(); col++) {
        cells_[row][col] = make_cell(row, col - map_->get_offset());
      };
    };

    diagonal_cost_ = sqrt(3.5);
  }

  PathFinder::~PathFinder() {
    // Delete cells
    for (int row = 0; row < map_->get_rows(); row++) {
      for (int col = 0; col < map_->get_cols(); col++) {
        delete cells_[row][col];
      };
    };
  }

  Cell* PathFinder::make_cell(int row, int col) {
    Cell* cell = new Cell;
    cell->cost = std::numeric_limits<double>::infinity();
    cell->path_cost = std::numeric_limits<double>::infinity();
    cell->visited = false;
    cell->previous = nullptr;
    cell->row = row;
    cell->col = col;
    return cell;
  }

  /*
   * Finds the cheapest path between start and end using the dijikstra algorithm.
   */
  std::vector<Eigen::Vector2f> PathFinder::find_path(Eigen::Vector2f from, Eigen::Vector2f to) {

    // Get pointers to start and end cells from current position to target.
    Eigen::Vector2f start_cell = map_->get_cell(from);
    Eigen::Vector2f end_cell = map_->get_cell(to);

    Cell* start = cells_[(int)start_cell.x()][(int)start_cell.y() + map_->get_offset()];
    Cell* end = cells_[(int)end_cell.x()][(int)end_cell.y() + map_->get_offset()];

    reset_cells();

    std::priority_queue<Cell> cell_queue;
    start->cost = 0;
    cell_queue.push(*start);
    Cell* curr_cell;
    std::set<Cell*> curr_neighbors;
    double alt_cost;
    while (!cell_queue.empty()) {
      Cell next_cell = cell_queue.top();
      curr_cell = cells_[next_cell.row][next_cell.col + map_->get_offset()];
      cell_queue.pop();
      if (curr_cell == end) {
        break;
      };
      curr_neighbors = get_neighbors(curr_cell);
      // Calculates and updates lowest costs and changes
      // the priority of neighbors if needed.
      for (Cell* neighbor : curr_neighbors) {
        alt_cost = curr_cell->cost + neighbor->path_cost;
        if (alt_cost < neighbor->cost) {
          neighbor->cost = alt_cost;
          neighbor->previous = curr_cell;
          if (neighbor->visited) {
            cell_queue = get_resorted_queue(cell_queue);
          } else {
            neighbor->visited = true;
            cell_queue.push(*neighbor);
          };
        };
      };
    }

    std::vector<Eigen::Vector2f> path = get_path(start, end);
    std::vector<Eigen::Vector2f> smooth = make_smooth(path);

    return smooth;
  }

  /*
   * Sort old_queue by reconstructing it and returning new_queue.
   */
  std::priority_queue<Cell> PathFinder::get_resorted_queue(std::priority_queue<Cell> old_queue) {
    std::priority_queue<Cell> new_queue;
    for (int i = 0; i < old_queue.size(); i++) {
      Cell cell = old_queue.top();
      new_queue.push(*cells_[cell.row][cell.col + map_->get_offset()]);
      old_queue.pop();
    };
    return new_queue;
  }

  /*
   * Reset Cells that have been used for path finding so they can be used again.
   */
  void PathFinder::reset_cells() {
    for (int row = 0; row < map_->get_rows(); row++) {
      for (int col = 0; col < map_->get_cols(); col++) {
        cells_[row][col]->cost = std::numeric_limits<double>::infinity();
        cells_[row][col]->visited = false;
        cells_[row][col]->previous = nullptr;
      };
    };
  }

  /*
   * Returns a set of neighbors of cell. A Neighbor is cell we can get to
   * from cell. In other words an adjacent cell with no wall in between.
   */
  std::set<Cell*> PathFinder::get_neighbors(Cell* cell) {
    std::set<Cell*> neighbors;
    Eigen::Vector2f current_cell(cell->row, cell->col);

    if (map_->is_north_reachable_from_cell(current_cell)) {
      Cell* north_neighbor = cells_[cell->row + 1][cell->col + map_->get_offset()];
      north_neighbor->path_cost = 1;
      neighbors.insert(north_neighbor);
    };

    if (map_->is_north_east_reachable_from_cell(current_cell)) {
      Cell* north_east_neighbor = cells_[cell->row + 1][cell->col - 1 + map_->get_offset()];
      north_east_neighbor->path_cost = diagonal_cost_;
      neighbors.insert(north_east_neighbor);
    };

    if (map_->is_east_reachable_from_cell(current_cell)) {
      Cell* east_neighbor = cells_[cell->row][cell->col - 1 + map_->get_offset()];
      east_neighbor->path_cost = 1;
      neighbors.insert(east_neighbor);
    };

    if (map_->is_south_east_reachable_from_cell(current_cell)) {
      Cell* south_east_neighbor = cells_[cell->row - 1][cell->col - 1 + map_->get_offset()];
      south_east_neighbor->path_cost = diagonal_cost_;
      neighbors.insert(south_east_neighbor);
    };

    if (map_->is_south_reachable_from_cell(current_cell)) {
      Cell* south_neighbor = cells_[cell->row - 1][cell->col + map_->get_offset()];
      south_neighbor->path_cost = 1;
      neighbors.insert(south_neighbor);
    };

    if (map_->is_south_west_reachable_from_cell(current_cell)) {
      Cell* south_west_neighbor = cells_[cell->row - 1][cell->col + 1 + map_->get_offset()];
      south_west_neighbor->path_cost = diagonal_cost_;
      neighbors.insert(south_west_neighbor);
    };

    if (map_->is_west_reachable_from_cell(current_cell)) {
      Cell* west_neighbor = cells_[cell->row][cell->col + 1 + map_->get_offset()];
      west_neighbor->path_cost = 1;
      neighbors.insert(west_neighbor);
    };

    if (map_->is_north_west_reachable_from_cell(current_cell)) {
      Cell* north_west_neighbor = cells_[cell->row + 1][cell->col + 1 + map_->get_offset()];
      north_west_neighbor->path_cost = diagonal_cost_;
      neighbors.insert(north_west_neighbor);
    };

    return neighbors;
  }

  /*
  * Backtracks from end to start and returns resulting path.
  */
  std::vector<Eigen::Vector2f> PathFinder::get_path(Cell* start, Cell* end) {
    float cell_center_x;
    float cell_center_y;
    std::vector<Eigen::Vector2f> path;
    bool foundEnd = end->previous != nullptr;
    if (foundEnd) {
      Cell* backtracker = end;
      while (backtracker != start) {
        cell_center_x = map_->get_cell_size()/2 + map_->get_cell_size() * backtracker->row;
        cell_center_y = map_->get_cell_size()/2 + map_->get_cell_size() * backtracker->col;
        path.push_back(Eigen::Vector2f(cell_center_x, cell_center_y));
        backtracker = backtracker->previous;
      };
      cell_center_x = map_->get_cell_size()/2 + map_->get_cell_size() * start->row;
      cell_center_y = map_->get_cell_size()/2 + map_->get_cell_size() * start->col;
      path.push_back(Eigen::Vector2f(cell_center_x, cell_center_y));
      reverse(path.begin(), path.end());
    }
    return path;
  }

  std::vector<Eigen::Vector2f> PathFinder::make_smooth(const std::vector<Eigen::Vector2f>& path) {
    if (path.size() < 3) {
      return path;
    }
    int resolution = 3;
    std::vector<Eigen::Vector2f> smooth;
    smooth.push_back(path[0]);
    for (int i = 0; i < path.size() - 2; i++) {
      for (int t = 0; t < resolution; t++) {
        float s = 0.2 * t / resolution;
        Eigen::Vector2f first = path[i+1] + (path[i] - path[i+1]).normalized() * (0.2 - s);
        Eigen::Vector2f second = path[i+1] + (path[i+2] - path[i+1]).normalized() * s;
        Eigen::Vector2f point = first + (second - first) * s / 0.2;
        smooth.push_back(point);
      }
    }
    smooth.push_back(path[path.size()-1]);

    return smooth;
  }
}
