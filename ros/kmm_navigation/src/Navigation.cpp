#include "kmm_navigation/Navigation.hpp"
#include "kmm_navigation/MoveToGoal.h"

namespace kmm_navigation {

  Navigation::Navigation(ros::NodeHandle nh)
  : nh_(nh),
    action_server_(nh_, "navigation", boost::bind(&Navigation::navigation_callback, this, _1), false) {
    // Publishers
    path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("path", 1);
    // Subscribers
    walls_sub_ = nh_.subscribe("walls", 1, &Navigation::walls_callback, this);
    position_sub_ = nh_.subscribe("position", 1, &Navigation::position_callback, this);
    target_position_sub_ = nh_.subscribe("target_position", 1, &Navigation::target_position_callback, this);

    // Get map variables
    if (!nh_.getParam("/map_rows", map_rows_)) {
        ROS_ERROR("Couldn't set map_rows!");
    }
    if (!nh_.getParam("/map_cols", map_cols_)) {
        ROS_ERROR("Couldn't set map_cols!");
    }
    if (!nh_.getParam("/cell_size", cell_size_)) {
        ROS_ERROR("Couldn't set cell_size!");
    }
    map_ = new Map(map_rows_, map_cols_, cell_size_);

    // Cells
    for (int row = 0; row < map_rows_; row++) {
      for (int col = 0; col < map_cols_; col++) {
        cells_[row][col] = make_cell(row, col - map_->offset_);
      };
    };

    action_server_.start();
  }

  Navigation::~Navigation() {
    // Delete cells
    for (int row = 0; row < map_rows_; row++) {
      for (int col = 0; col < map_cols_; col++) { // -25 <= col <= 25, extra 25 added to make non-negative!
        delete cells_[row][col];
      };
    };
    delete map_;
  }

  void Navigation::navigation_callback(const kmm_navigation::MoveToGoalConstPtr &goal) {
    ROS_INFO("Got new navigation request to x: %.2f, y: %.2f, angle: %.2f", goal->x, goal->y, goal->angle);

    ros::Rate rate(3); // 3 Hz

    ROS_INFO("Räkna ut path med dijikstra...");

    while (true) {
      ROS_INFO("Gör aktiv reglering för att följa pathen...");

      if (action_server_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("Oj, navigeringen avbröts av klienten!");
        action_server_.setPreempted();
        return;
      }

      rate.sleep();
    }

    ROS_INFO("Roboten åkte hela vägen fram till target!");
    action_server_.setSucceeded(result_);
  }

  void Navigation::walls_callback(std_msgs::Int8MultiArray msg) {
    for (int i = 0; i < map_->walls_size_; i++) {
      map_->walls_[i] = msg.data[i];
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
    Eigen::Vector2f start_cell = map_->get_cell(pos_);
    Eigen::Vector2f end_position(msg.linear.x, msg.linear.y);
    Eigen::Vector2f end_cell = map_->get_cell(end_position);
    Cell* start = cells_[(int)start_cell.x()][(int)start_cell.y() + map_->offset_];
    Cell* end = cells_[(int)end_cell.x()][(int)end_cell.y() + map_->offset_];
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
      curr_cell = cells_[next_cell.row][next_cell.col + map_->offset_];
      cell_queue.pop();
      if (curr_cell == end) {
        break;
      };
      curr_neighbors = get_neighbors(curr_cell);
      // Calculates and updates lowest costs and changes
      // the priority of neighbors if needed.
      alt_cost = curr_cell->cost + 1;
      for (Cell* neighbor : curr_neighbors) {
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
  std::priority_queue<Cell> Navigation::get_resorted_queue(std::priority_queue<Cell> old_queue) {
    std::priority_queue<Cell> new_queue;
    for (int i = 0; i < old_queue.size(); i++) {
      Cell cell = old_queue.top();
      new_queue.push(*cells_[cell.row][cell.col + map_->offset_]);
      old_queue.pop();
    };
    return new_queue;
  }

  /*
   * Reset Cells that have been used for path finding so they can be used again.
   */
  void Navigation::reset_cells() {
    for (int row = 0; row < map_rows_; row++) {
      for (int col = 0; col < map_cols_; col++) {
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
    bool cell_above_reachable = !map_->is_wall_above_cell(cell_vector) && cell->row < (map_rows_ - 1);
    if (cell_above_reachable) {
      Cell* above_neighbor = cells_[cell->row + 1][cell->col + map_->offset_];
      neighbors.insert(above_neighbor);
    };
    bool cell_below_reachable = !map_->is_wall_below_cell(cell_vector) && cell->row > 0;
    if (cell_below_reachable) {
      Cell* below_neighbor = cells_[cell->row - 1][cell->col + map_->offset_];
      neighbors.insert(below_neighbor);
    };
    bool cell_left_reachable = !map_->is_wall_left_cell(cell_vector) && cell->col < map_->offset_;
    if (cell_left_reachable) {
      Cell* left_neighbor = cells_[cell->row][cell->col + 1 + map_->offset_];
      neighbors.insert(left_neighbor);
    };
    bool cell_right_reachable = !map_->is_wall_right_cell(cell_vector) && cell->col > -map_->offset_;
    if (cell_right_reachable) {
      Cell* right_neighbor = cells_[cell->row][cell->col - 1 + map_->offset_];
      neighbors.insert(right_neighbor);
    };
    return neighbors;
  }

  /*
  * Backtracks from end to start and returns resulting path.
  */
  std::vector<Eigen::Vector2f> Navigation::get_path(Cell* start, Cell* end) {
    float cell_center_x;
    float cell_center_y;
    std::vector<Eigen::Vector2f> path;
    bool foundEnd = end->previous != nullptr;
    if (foundEnd) {
      Cell* backtracker = end;
      while (backtracker != start) {
        cell_center_x = cell_size_/2 + cell_size_ * backtracker->row;
        cell_center_y = cell_size_/2 + cell_size_ * backtracker->col;
        path.push_back(Eigen::Vector2f(cell_center_x, cell_center_y));
        backtracker = backtracker->previous;
      };
      cell_center_x = cell_size_/2 + cell_size_ * start->row;
      cell_center_y = cell_size_/2 + cell_size_ * start->col;
      path.push_back(Eigen::Vector2f(cell_center_x, cell_center_y));
      reverse(path.begin(), path.end());
    }
    return path;
  }

  std::vector<Eigen::Vector2f> Navigation::make_smooth(const std::vector<Eigen::Vector2f>& path) {
    if (path.size() < 3) {
      return path;
    }
    int resolution = 10;
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

  void Navigation::publish_path(std::vector<Eigen::Vector2f> path) {
    path_msg_.poses.clear();
    for (Eigen::Vector2f& p : path) {
      geometry_msgs::Pose pose;
      geometry_msgs::Point cell;
      cell.x = p.x();
      cell.y = p.y();
      cell.z = 0;
      pose.position = cell;
      path_msg_.poses.push_back(pose);
    }
    path_pub_.publish(path_msg_);
    return;
  }
}
