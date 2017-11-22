#include "kmm_navigation/Navigation.hpp"

namespace kmm_navigation {

  Navigation::Navigation(ros::NodeHandle nh)
  : nh_(nh), map_(53) {
    // Publishers
    path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("path", 1);

    // Subscribers
    wall_array_sub_ = nh_.subscribe("wall_array", 1, &Navigation::wall_array_callback, this);
    position_sub_ = nh_.subscribe("position", 1, &Navigation::position_callback, this);
    target_position_sub_ = nh_.subscribe("target_position", 1, &Navigation::target_position_callback, this);

    // Cells
    for (int row = 0; row <= 25; row++) {
      for (int col = 0; col <= 50; col++) { // -25 <= col <= 25, extra 25 added to make non-negative!
        cells_[row][col] = make_cell(row, col - 25);
      };
    };
  }

  Navigation::~Navigation() {
    // Delete cells
    for (int row = 0; row <= 25; row++) {
      for (int col = 0; col <= 50; col++) { // -25 <= col <= 25, extra 25 added to make non-negative!
        delete cells_[row][col];
      };
    };
  }

  void Navigation::wall_array_callback(std_msgs::Int8MultiArray msg) {
    for (int i = 0; i < 1500; i++) {
      map_.wall_arr_[i] = msg.data[i];
    };
    return;
  }

  void Navigation::position_callback(geometry_msgs::PoseWithCovarianceStamped msg) {
    Eigen::Vector2f pos(msg.pose.pose.position.x, msg.pose.pose.position.y);
    pos_[0] = pos.x();
    pos_[1] = pos.y();
    return;
  }

  Cell* Navigation::make_cell(int row, int col) {
    Cell* cell = new Cell;
    cell->cost = std::numeric_limits<double>::infinity();
    cell->visited = false;
    cell->previous = nullptr;
    cell->row = row;
    cell->col = col;
    return cell;
  }

  void Navigation::target_position_callback(geometry_msgs::Twist msg) {
    // Get pointers to start and end cells from current position to target.
    Eigen::Vector2f start_cell = map_.get_cell(pos_);
    Eigen::Vector2f end_position(msg.linear.x, msg.linear.y);
    Eigen::Vector2f end_cell = map_.get_cell(end_position);
    Cell* start = cells_[(int)start_cell.x()][(int)start_cell.y() + 25];
    Cell* end = cells_[(int)end_cell.x()][(int)end_cell.y() + 25];
    // Find path
    std::vector<Eigen::Vector2f> path = find_path(start, end);
    publish_path(path);
    return;
  }

  /*
   * Finds the cheapest path between start and end using the dijikstra algorithm.
   */
  std::vector<Eigen::Vector2f> Navigation::find_path(Cell* start, Cell* end) {
    reset_cells();
    std::priority_queue<Cell> cell_queue;
    start->cost = 0;
    cell_queue.push(*start);
    Cell* curr_cell;
    std::set<Cell*> curr_neighbors;
    double alt_cost;
    while (!cell_queue.empty()) {
      Cell next_cell = cell_queue.top();
      curr_cell = cells_[next_cell.row][next_cell.col + 25];
      cell_queue.pop();
      if (curr_cell == end) {
        break;
      };
      curr_neighbors = get_neighbors(curr_cell);
      // Calculates and updates lowest costs and changes
      // the priority of neighbors if needed.
      alt_cost = curr_cell->cost + 1;
      for (std::set<Cell*>::iterator neighbor_it = curr_neighbors.begin();
         neighbor_it != curr_neighbors.end(); neighbor_it++) {
        if (alt_cost < (*neighbor_it)->cost) {
          (*neighbor_it)->cost = alt_cost;
          (*neighbor_it)->previous = curr_cell;
          if ((*neighbor_it)->visited) {
            cell_queue = get_resorted_queue(cell_queue);
          } else {
            (*neighbor_it)->visited = true;
            cell_queue.push(**neighbor_it);
          };
        };
      };
    }
    return get_path(start, end);
  }

  /*
   * Sort old_queue by reconstructing it and returning new_queue.
   */
  std::priority_queue<Cell> Navigation::get_resorted_queue(std::priority_queue<Cell> old_queue) {
    std::priority_queue<Cell> new_queue;
    for (int i = 0; i < old_queue.size(); i++) {
      Cell cell = old_queue.top();
      new_queue.push(*cells_[cell.row][cell.col + 25]);
      old_queue.pop();
    };
    return new_queue;
  }

  /*
   * Reset Cells that have been used for path finding so they can be used again.
   */
  void Navigation::reset_cells() {
    for (int row = 0; row <= 25; row++) {
      // -25 <= col <= 25, extra 25 added to make non-negative!
      for (int col = 0; col <= 50; col++) {
        cells_[row][col]->cost = std::numeric_limits<double>::infinity();
        cells_[row][col]->visited = false;
        cells_[row][col]->previous = nullptr;
      };
    };
    return;
  }

  /*
   * Returns a set of neighbors of cell. A Neighbor is cell we can get to
   * from cell. In other words an adjacent cell with no wall in between.
   */
  std::set<Cell*> Navigation::get_neighbors(Cell* cell) {
    std::set<Cell*> neighbors;
    Eigen::Vector2f cell_vector(cell->row, cell->col);
    if (!map_.is_wall_above_cell(cell_vector) && cell->row < 25) {
      neighbors.insert(cells_[cell->row + 1][cell->col + 25]);
    };
    if (!map_.is_wall_below_cell(cell_vector) && cell->row > 0) {
      neighbors.insert(cells_[cell->row - 1][cell->col + 25]);
    };
    if (!map_.is_wall_left_cell(cell_vector) && cell->col < 25) {
      neighbors.insert(cells_[cell->row][cell->col + 1 + 25]);
    };
    if (!map_.is_wall_right_cell(cell_vector) && cell->col > -25) {
      neighbors.insert(cells_[cell->row][cell->col - 1 + 25]);
    };
    return neighbors;
  }

  /*
  * Backtracks from end to start and returns resulting path.
  */
  std::vector<Eigen::Vector2f> Navigation::get_path(Cell* start, Cell* end) {
    std::vector<Eigen::Vector2f> path;
    bool foundEnd = end->previous != nullptr;
    if (foundEnd) {
      Cell* backtracker = end;
      while (backtracker != start) {
          path.push_back(Eigen::Vector2f(backtracker->row, backtracker->col));
          backtracker = backtracker->previous;
      }
      path.push_back(Eigen::Vector2f(start->row, start->col));
      reverse(path.begin(), path.end());
    }
    return path;
  }

  void Navigation::publish_path(std::vector<Eigen::Vector2f> path) {
    path_msg_.poses.clear();
    for (int i = 0; i < path.size(); i++) {
      geometry_msgs::Pose pose;
      geometry_msgs::Point cell;
      cell.x = path[i].x();
      cell.y = path[i].y();
      cell.z = 0;
      pose.position = cell;
      path_msg_.poses.push_back(pose);
    }
    path_pub_.publish(path_msg_);
    return;
  }
}
