#include "kmm_navigation/Navigation.hpp"

namespace kmm_navigation {

  Navigation::Navigation(ros::NodeHandle nh)
  : nh_(nh), map_(53) {
    // Publishers
    path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("path", 1);

    // Subscribers
    wall_array_sub_ = nh_.subscribe("wall_array", 1, &Navigation::wall_array_callback, this);
    position_sub_ = nh_.subscribe("position", 1, &Navigation::position_callback, this);

    // Cells
    for (int row = 0; row <= 25; row++) {
      for (int col = 0; col <= 50; col++) { // -25 <= col <= 25, extra 25 added to make non-negative!
        cells_[row][col] = make_cell(row, col - 25);
      };
    };
  }

  Navigation::~Navigation() {
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

    Eigen::Vector2f cell = map_.get_cell(pos);
    bool above = map_.is_wall_above_cell(cell);
    bool below = map_.is_wall_below_cell(cell);
    bool right = map_.is_wall_right_cell(cell);
    bool left = map_.is_wall_left_cell(cell);
    ROS_INFO_THROTTLE(1, "----- POS: (%f,%f) -----", pos.x(), pos.y());
    ROS_INFO_THROTTLE(1, "----- CELL: (%f,%f) -----", cell.x(), cell.y());
    if (above) {
      ROS_INFO_THROTTLE(1, "CELL ABOVE!");
    };
    if (below) {
      ROS_INFO_THROTTLE(1, "CELL BELOW");
    };
    if (right) {
      ROS_INFO_THROTTLE(1, "CELL RIGHT");
    };
    if (left) {
      ROS_INFO_THROTTLE(1, "CELL LEFT");
    };
    return;
  }

  Cell* Navigation::make_cell(int row, int col) {
    Cell cell;
    cell.cost = std::numeric_limits<double>::infinity();
    cell.visited = false;
    cell.row = row;
    cell.col = col;
    Cell* cell_ptr = &cell;
    return cell_ptr;
  }

  void Navigation::target_position_callback(geometry_msgs::Twist msg) {
    // Find path from current postion to target position
    Eigen::Vector2f start_cell = map_.get_cell(pos_);
    Eigen::Vector2f end_position(msg.linear.x, msg.linear.y);
    Eigen::Vector2f end_cell = map_.get_cell(end_position);
    Cell* start = cells_[(int)start_cell.x()][(int)start_cell.y() + 25];
    Cell* end = cells_[(int)end_cell.x()][(int)end_cell.y() + 25];
    std::vector<Eigen::Vector2f> path = find_path(start, end);
    // Construct message
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

  /*
 * Finds the cheapest path between start and end using the dijikstra algorithm.
 */
std::vector<Eigen::Vector2f> Navigation::find_path(Cell* start, Cell* end) {
    reset_cells();
    std::priority_queue<Cell*> cell_queue;
    start->cost = 0;
    cell_queue.push(start);
    Cell* curr_cell;
    std::set<Cell*> curr_neighbors;
    double alt_cost;
    while (!cell_queue.empty()) {
        curr_cell = cell_queue.top();
        cell_queue.pop();
        if (curr_cell == end) {
            break;
        };
        curr_neighbors = get_neighbors(curr_cell); // TODO write this
        // Calculates and updates lowest costs and changes
        // the priority of neighbors if needed.
        for (std::set<Cell*>::iterator neighbor_it = curr_neighbors.begin();
             neighbor_it != curr_neighbors.end(); neighbor_it++) {
            alt_cost = curr_cell->cost + 1;
            if (alt_cost < (*neighbor_it)->cost) {
                (*neighbor_it)->cost = alt_cost;
                (*neighbor_it)->previous = curr_cell;
                if ((*neighbor_it)->visited) {
                  cell_queue = get_resorted_queue(cell_queue);
                } else {
                  (*neighbor_it)->visited = true;
                  cell_queue.push(*neighbor_it);
                };
            };
        };
    }
    return get_path(start, end);
  }

  std::priority_queue<Cell*> Navigation::get_resorted_queue(std::priority_queue<Cell*> old_queue) {
    std::priority_queue<Cell*> new_queue;
    for (int i = 0; i < old_queue.size(); i++) {
      new_queue.push(old_queue.top());
      old_queue.pop();
    };
  }

  void Navigation::reset_cells() {
    for (int row = 0; row <= 25; row++) {
      for (int col = 0; col <= 50; col++) { // -25 <= col <= 25, extra 25 added to make non-negative!
        cells_[row][col]->cost = std::numeric_limits<int>::infinity();
        cells_[row][col]->visited = false;
      };
    };
    return;
  }

  std::set<Cell*> Navigation::get_neighbors(Cell* cell) {
    std::set<Cell*> neighbors;
    Eigen::Vector2f cell_vector(cell->row, cell->col);
    if (!map_.is_wall_above_cell(cell_vector) && cell->row < 25) {
      Cell* cell_above = cells_[cell->row + 1][cell->col + 25];
      neighbors.insert(cell_above);
    };
    if (!map_.is_wall_below_cell(cell_vector) && cell->row > 0) {
      Cell* cell_below = cells_[cell->row - 1][cell->col + 25];
      neighbors.insert(cell_below);
    };
    if (!map_.is_wall_left_cell(cell_vector) && cell->col < 25) {
      Cell* cell_left = cells_[cell->row][cell->col + 1 + 25];
      neighbors.insert(cell_left);
    };
    if (!map_.is_wall_right_cell(cell_vector) && cell->col > -25) {
      Cell* cell_right = cells_[cell->row][cell->col - 1 + 25];
      neighbors.insert(cell_right);
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
}
