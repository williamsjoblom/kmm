#include "kmm_mapping/Mapping.hpp"

namespace kmm_mapping {

  Mapping::Mapping(ros::NodeHandle nh)
  : nh_(nh),
    action_server_(nh_, "remove_walls", boost::bind(&Mapping::remove_walls_callback, this, _1), false)
  {
    // Publishers
    mapping_pub_ = nh_.advertise<std_msgs::Bool>("mapping", 1);
    walls_pub_ = nh_.advertise<std_msgs::Int8MultiArray>("walls", 1);
    end_points_pub_ = nh_.advertise<sensor_msgs::PointCloud>("end_points", 1);

    // Subscribers
    mapping_scan_sub_ = nh_.subscribe("mapping_scan", 1, &Mapping::mapping_scan_callback, this);

    // Timers
    publish_mapping_timer_ = nh_.createTimer(ros::Duration(1. / 5), &Mapping::publish_mapping, this);
    publish_walls_timer_ = nh_.createTimer(ros::Duration(1. / 5), &Mapping::publish_walls, this);
    publish_end_points_timer_ = nh_.createTimer(ros::Duration(1. / 5), &Mapping::publish_end_points, this);

    // Services
    mapping_service_ = nh_.advertiseService("set_mapping", &Mapping::set_mapping, this);
    reset_map_service_ = nh_.advertiseService("reset_map", &Mapping::reset_map, this);

    // Get map variables
    if (!nh_.getParam("/map_rows", h_)) {
        ROS_ERROR("Couldn't set map_rows!");
    }
    if (!nh_.getParam("/map_cols", w_)) {
        ROS_ERROR("Couldn't set map_cols!");
    }
    if (!nh_.getParam("/cell_size", cell_size_)) {
        ROS_ERROR("Couldn't set cell_size!");
    }
    offset_ = (w_ - 1) / 2;
    walls_size_ = (w_ + (w_ + 1)) * h_ + w_;

    // Enable mapping_
    mapping_ = true;

    // Initialize wall vector with 0's
		for (int i = 0; i < walls_size_; i++) {
			walls_.push_back(0);
    };

    // Set dimensions of wall array in message
    walls_msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
    walls_msg_.layout.dim[0].size = walls_.size();
    walls_msg_.layout.dim[0].stride = 1;
    walls_msg_.layout.dim[0].label = "x";

    // Wall point count requirements
    pnt_cnt_req_ = 5;
    times_req_ = 5;

    // Start the action server when all other
    // instance variables are initiated.
    action_server_.start();
  }

  Mapping::~Mapping() {
  }

  void Mapping::set_pnt_cnt_req(int pnt_cnt_req) {
    pnt_cnt_req_ = pnt_cnt_req;
  }

  void Mapping::set_times_req(int times_req) {
    times_req_ = times_req;
  }

  /* Analyzes points by counting number of collisions on the same wall.
   * Finally publishes the walls that have enough collisions.
   */
  void Mapping::mapping_scan_callback(const sensor_msgs::PointCloud::ConstPtr& msg) {
    if (mapping_) {
      std::vector<Eigen::Vector2f> wall_points;
      for (const auto &p : msg->points) {
        Eigen::Vector2f point;
        point[0] = p.x;
        point[1] = p.y;
        wall_points.push_back(point);
      };
      // Iterate through wall points.
      for (Eigen::Vector2f& wall_point: wall_points) {
        float x = wall_point.x();
        float y = wall_point.y();
        float eps = 0.000001;
        float rem_x = std::remainder(std::fabs(x), cell_size_);
        float rem_y = std::remainder(std::fabs(y), cell_size_);
        bool on_horizontal_wall = (rem_x > -eps) && (rem_x < eps);
        bool on_vertical_wall = (rem_y > -eps) && (rem_y < eps);
        if (on_horizontal_wall) {
          int row = std::round(x / cell_size_);
          int col = std::ceil(std::fabs(y) / cell_size_) * (y < 0 ? -1 : 1); // col can never be 0
          if (!is_horizontal_wall_at(row, col)) {
            increment_horizontal_wall_point_count(row, col);
          }
        } else if (on_vertical_wall) {
          int row = std::ceil(std::fabs(x) / cell_size_) * (x < 0 ? -1 : 1); // row can never be 0
          int col = std::round(y / cell_size_);
          if (!is_vertical_wall_at(row, col)) {
            increment_vertical_wall_point_count(row, col);
          }
        } else {
          ROS_INFO("Unable to determine wall position! x: %f y: %f fmod x: %f fmod y: %f",
            x, y, std::remainder(std::fabs(x), cell_size_),std::remainder(std::fabs(y), 0.4));
        };
      };
      reset_wall_point_counts();
    }
    return;
  }


  /*
   * WALL POINT COUNT FUNCTIONS
   */


  /* Create new WallPointCount struct */
  WallPointCount Mapping::make_wall_point_count(int row, int col) {
    Eigen::Vector2f pos(row, col);
    WallPointCount wall_point_count;
    wall_point_count.position = pos;
    wall_point_count.pnt_cnt = 1; // Initially one because it is added when point is found
    wall_point_count.times = 0;
    wall_point_count.found = true; // Initially true because it is only added when found
    wall_point_count.added = false;
    return wall_point_count;
  }

  /*
   * Set all wall point counts to 0. Also set times to 0 if wall wasn't
   * found (meaning enough points were on it) during this scan.
   * Finally reset found to false for next scan.
   */
  void Mapping::reset_wall_point_counts() {
    // Reset horizontal wall point counts
    std::vector<WallPointCount> new_hor_wall_point_counts;
    for (WallPointCount& wall_point_count : hor_wall_point_counts_) {
      if (!wall_point_count.added) { // Drop wall count if already added
        wall_point_count.pnt_cnt = 0;
        if (!wall_point_count.found) {
          wall_point_count.times = 0;
        }
        wall_point_count.found = false;
        new_hor_wall_point_counts.push_back(wall_point_count);
      }
    };
    hor_wall_point_counts_ = new_hor_wall_point_counts;

    // Reset vertical wall point counts
    std::vector<WallPointCount> new_ver_wall_point_counts;
    for (WallPointCount& wall_point_count : ver_wall_point_counts_) {
      if (!wall_point_count.added) { // Drop wall count if already added
        wall_point_count.pnt_cnt = 0;
        if (!wall_point_count.found) {
          wall_point_count.times = 0;
        }
        wall_point_count.found = false;
        new_ver_wall_point_counts.push_back(wall_point_count);
      }
    };
    ver_wall_point_counts_ = new_ver_wall_point_counts;
  }

  void Mapping::increment_horizontal_wall_point_count(int row, int col) {
    return increment_wall_point_count(hor_wall_point_counts_, true, row, col);
  }

  void Mapping::increment_vertical_wall_point_count(int row, int col) {
    return increment_wall_point_count(ver_wall_point_counts_, false, row, col);
  }

  /*
   * Increment the wall point count (pnt_cnt) of a wall.
   * Creates a new one with pnt_cnt 1 if WallPointCount is missing.
   * If pnt_cnt gets high enough, increment times.
   * If times gets high enough, add wall and update end points.
   */
  void Mapping::increment_wall_point_count(
      std::vector<WallPointCount>& wall_point_counts,
      bool horizontal, int row, int col) {
    int existing_wall_row;
    int existing_wall_col;
    for (WallPointCount& wall_point_count : wall_point_counts) {
      existing_wall_row = wall_point_count.position[0];
      existing_wall_col = wall_point_count.position[1];
      bool found_existing_wall_point_count = (existing_wall_row == row) && (existing_wall_col == col);
      if (found_existing_wall_point_count && !wall_point_count.added) {
        wall_point_count.pnt_cnt++;
        if (wall_point_count.pnt_cnt == pnt_cnt_req_) {
          wall_point_count.found = true;
          wall_point_count.times++;
          if (wall_point_count.times == times_req_) {
            wall_point_count.added = true; // ?
            add_wall_at(row, col, horizontal); // ?
            update_end_points(row, col, horizontal);
          };
        };
        return;
      };
    };
    wall_point_counts.push_back(make_wall_point_count(row, col));
    return;
  }


  /*
   * WALL FUNCTIONS
   */


  /*
   * Add wall to walls_.
   */
  void Mapping::add_wall_at(int row, int col, bool horizontal) {
    if (horizontal) {
      int t = (col >= 1 ? 1 : 0);
      walls_[row*w_ + row*(w_ + 1) + offset_ + col - t] = 1;
    } else {
      walls_[row*w_ + (w_ + 1)*(row - 1) + offset_ + col] = 1;
    };
  }

  /*
   * Action server callback. Gets an end point that kmm_exploration has
   * deemed unreachable. Removes it and all walls connected to it.
  */
  void Mapping::remove_walls_callback(const kmm_mapping::RemoveWallsGoalConstPtr &end_point) {
    Eigen::Vector2f e(end_point->x, end_point->y);
    toggle_end_point(e);
    remove_walls_at_crossing(e);
    action_server_.setSucceeded(result_);
  }

  /*
   * Removes all the walls that connect to the point crossing.
   * End points associated with removed walls will be updated.
   */
  void Mapping::remove_walls_at_crossing(Eigen::Vector2f crossing) {
    remove_wall_north_of_crossing(crossing);
    remove_wall_east_of_crossing(crossing);
    remove_wall_south_of_crossing(crossing);
    remove_wall_west_of_crossing(crossing);
  }

  /*
   * Removes the wall north of crossing.
   * End point north of crossing is updated.
   */
  void Mapping::remove_wall_north_of_crossing(Eigen::Vector2f crossing) {
    int cs_mults_x = get_num_cell_size_multiples(crossing.x());
    int cs_mults_y = get_num_cell_size_multiples(crossing.y());

    int row = cs_mults_x + 1;
    int col = cs_mults_y;

    int index = get_vertical_wall_index(row, col);
    if (is_wall_index_within_bounds(index) && is_vertical_wall_at(row, col)) {
      walls_[index] = 0;
      toggle_end_point(get_north_end_point(crossing));
    }
  }

  /*
   * Removes the wall east of crossing.
   * End point east of crossing is updated.
   */
  void Mapping::remove_wall_east_of_crossing(Eigen::Vector2f crossing) {
    int cs_mults_x = get_num_cell_size_multiples(crossing.x());
    int cs_mults_y = get_num_cell_size_multiples(crossing.y());

    int row = cs_mults_x;

    int t = (cs_mults_y <= 0 ? 1 : 0);
    int col = cs_mults_y - t;

    int index = get_horizontal_wall_index(row, col);
    if (is_wall_index_within_bounds(index) && is_horizontal_wall_at(row, col)) {
      walls_[index] = 0;
      toggle_end_point(get_east_end_point(crossing));
    }
  }

  /*
   * Removes the wall south of crossing.
   * End point south of crossing is updated.
   */
  void Mapping::remove_wall_south_of_crossing(Eigen::Vector2f crossing) {
    int cs_mults_x = get_num_cell_size_multiples(crossing.x());
    int cs_mults_y = get_num_cell_size_multiples(crossing.y());

    int row = cs_mults_x;
    int col = cs_mults_y;

    int index = get_vertical_wall_index(row, col);
    if (is_wall_index_within_bounds(index) && is_vertical_wall_at(row, col)) {
      walls_[index] = 0;
      toggle_end_point(get_south_end_point(crossing));
    }
  }

  /*
   * Removes the wall west of crossing.
   * End point west of crossing is updated.
   */
  void Mapping::remove_wall_west_of_crossing(Eigen::Vector2f crossing) {
    int cs_mults_x = get_num_cell_size_multiples(crossing.x());
    int cs_mults_y = get_num_cell_size_multiples(crossing.y());

    int row = cs_mults_x;

    int t = (cs_mults_y >= 0 ? 1 : 0);
    int col = cs_mults_y + t;

    int index = get_horizontal_wall_index(row, col);
    if (is_wall_index_within_bounds(index) && is_horizontal_wall_at(row, col)) {
      walls_[index] = 0;
      toggle_end_point(get_west_end_point(crossing));
    }
  }

  bool Mapping::is_horizontal_wall_at(int row, int col) {
    return is_wall_at(row, col, true);
  }

  bool Mapping::is_vertical_wall_at(int row, int col) {
    return is_wall_at(row, col, false);
  }

  /*
   * Returns true if there is a wall at given row, col.
   * Set horizontal = true if horizontal wall, else false.
   */
  bool Mapping::is_wall_at(int row, int col, bool horizontal) {
    if (horizontal) {
      int t = (col >= 1 ? 1 : 0);
      return walls_[row*w_ + row*(w_ + 1) + offset_ + col - t];
    } else {
      return walls_[row*w_ + (w_ + 1)*(row - 1) + offset_ + col];
    };
  }

  /*
   * Returns the wall index of horizontal wall
   */
  int Mapping::get_horizontal_wall_index(int row, int col) {
    return get_wall_index(row, col, true);
  }

  /*
   * Returns the wall index of vertical wall
   */
  int Mapping::get_vertical_wall_index(int row, int col) {
    return get_wall_index(row, col, false);
  }

  /*
   * Returns the wall index of wall
   */
  int Mapping::get_wall_index(int row, int col, int horizontal) {
    if (horizontal) {
      if (col == 0 || row < 0) { // Illegal row for horizontal
        return -1;
      };
      int t = (col >= 1 ? 1 : 0);
      return row*w_ + row*(w_ + 1) + offset_ + col - t;
    } else {
      if (row == 0) { // Illegal row for vertical
        return -1;
      };
      return row*w_ + (w_ + 1)*(row - 1) + offset_ + col;
    };
  }

  /*
   * Returns true if the wall index is within bounds, otherwise false
   */
  bool Mapping::is_wall_index_within_bounds(int wall_index) {
   return (0 <= wall_index && wall_index < walls_size_);
  }


  /*
   * END POINTS FUNCTIONS
   */


  /*
  * Calculates the two end points associated with wall and toggles them.
  */
  void Mapping::update_end_points(int row, int col, bool horizontal) {
    Eigen::Vector2f end_point_1;
    Eigen::Vector2f end_point_2;
    if (horizontal) {
      end_point_1[0] = row * cell_size_;
      end_point_1[1] = (col - 1*(col >= 1 ? 1 : 0)) * cell_size_;

      end_point_2[0] = end_point_1[0];
      end_point_2[1] = end_point_1[1] + cell_size_;
    } else {
      end_point_1[0] = (row - 1) * cell_size_;
      end_point_1[1] = col * cell_size_;

      end_point_2[0] = end_point_1[0] + cell_size_;
      end_point_2[1] = end_point_1[1];
    }
    toggle_end_point(end_point_1);
    toggle_end_point(end_point_2);
  }

  /*
   * Adds end_point to end_points_ if it wasn't already there.
   * Removes end_point from end_points_ if it was already there.
   */
  void Mapping::toggle_end_point(Eigen::Vector2f end_point) {
    float eps = 0.000001;
    float x = end_point.x();
    float y = end_point.y();
    bool found_end_point = false;
    for (auto it = end_points_.begin(); it < end_points_.end(); ) {
      float diff_x = fabs((*it).x() - x);
      float diff_y = fabs((*it).y() - y);
      bool end_point_equal = (diff_x < eps) && (diff_y < eps);
      if (end_point_equal) {
        it = end_points_.erase(it);
        found_end_point = true;
        break;
      }
      it++;
    };
    if (!found_end_point) {
      end_points_.push_back(end_point);
    }
  }

  /*
   * Returns the end point north of crossing
   */
  Eigen::Vector2f Mapping::get_north_end_point(Eigen::Vector2f crossing) {
    Eigen::Vector2f north(crossing.x() - cell_size_, crossing.y());
    return north;
  }

  /*
   * Returns the end point south of crossing
   */
  Eigen::Vector2f Mapping::get_south_end_point(Eigen::Vector2f crossing) {
    Eigen::Vector2f south(crossing.x() + cell_size_, crossing.y());
    return south;
  }

  /*
   * Returns the end point west of crossing
   */
  Eigen::Vector2f Mapping::get_west_end_point(Eigen::Vector2f crossing) {
    Eigen::Vector2f west(crossing.x(), crossing.y() + cell_size_);
    return west;
  }

  /*
   * Returns the end point east of crossing
   */
  Eigen::Vector2f Mapping::get_east_end_point(Eigen::Vector2f crossing) {
    Eigen::Vector2f east(crossing.x(), crossing.y() - cell_size_);
    return east;
  }


  /*
   * GENERAL HELP FUNCTIONS
   */


   /*
    * Return the number of times cell_size_ goes in float f.
    */
   int Mapping::get_num_cell_size_multiples(float f) {
     int times = 0;
     float rem = fabs(f);
     while (rem >= cell_size_) {
       rem -= cell_size_;
       times++;
     }
     return times * (std::signbit(f) ? -1 : 1); // Return times with the sign of f.
   }

  /*
   * Return true if vector1 == vector2, otherwise false
   */
  bool Mapping::are_equal(Eigen::Vector2f vector1, Eigen::Vector2f vector2){
    float eps = 0.000001;
    float diff_x = fabs(vector1.x() - vector2.x());
    float diff_y = fabs(vector1.y() - vector2.y());
    bool point_equal = (diff_x < eps) && (diff_y < eps);
    return point_equal;
  }


  /*
   * RESET/SET FUNCTIONS CALLED FROM GUI
   */


  /*
   * Callback for service requests to set mapping bool.
   */
  bool Mapping::set_mapping(std_srvs::SetBool::Request &req,
         std_srvs::SetBool::Response &res) {
    mapping_ = req.data;
    return true;
  }

  /*
   * Callback for service requests to reset map.
   */
  bool Mapping::reset_map(std_srvs::SetBool::Request &req,
    std_srvs::SetBool::Response &res) {
    // Reset walls
    walls_.clear();
    for (int i = 0; i < walls_size_; i++) {
			walls_.push_back(0);
    };

    // Reset wall point counts
    hor_wall_point_counts_.clear();
    ver_wall_point_counts_.clear();

    end_points_.clear();

    return true;
  }


  /*
   * PUBLISHING FUNCTIONS
   */


  void Mapping::publish_mapping(const ros::TimerEvent&) {
    std_msgs::Bool mapping_msg;
    mapping_msg.data = mapping_;
    mapping_pub_.publish(mapping_msg);
  }

  void Mapping::publish_walls(const ros::TimerEvent&) {
    walls_msg_.data.clear();
    for (int& is_wall : walls_) {
        walls_msg_.data.push_back(is_wall);
    }
    walls_pub_.publish(walls_msg_);
  }

  void Mapping::publish_end_points(const ros::TimerEvent&) {
    end_points_msg_.points.clear();
    for (Eigen::Vector2f end_point : end_points_) {
      geometry_msgs::Point32 point;
      point.x = end_point.x();
      point.y = end_point.y();
      point.z = 0;
      end_points_msg_.points.push_back(point);
    };
    end_points_pub_.publish(end_points_msg_);
  }
}




/*
 * Counts the number of walls that connects at the point crossing, and returns
 * this value.
 */
/*int Mapping::count_connecting_walls_at(Eigen::Vector2f crossing) {
  int count = 0;
  Eigen::Vector2f south = get_south_end_point(crossing);
  Eigen::Vector2f east = get_east_end_point(crossing);
  int north_wall = convert_to_wall_index(crossing, false);
  int south_wall = convert_to_wall_index(south, false);
  int east_wall = convert_to_wall_index(east, true);
  int west_wall = convert_to_wall_index(crossing, true);
  if (wall_exists(north_wall)) {
    count++;
  }
  if (wall_exists(south_wall)) {
    count++;
  }
  if (wall_exists(east_wall)) {
    count++;
  }
  if (wall_exists(west_wall)) {
    count++;
  }
  return count;
}*/

/*
 * Removes end_point from end_points_ (if it exists).
*/
/*void Mapping::remove_end_point(Eigen::Vector2f end_point) {
  float eps = 0.000001;
  float x = end_point.x();
  float y = end_point.y();
  for (auto it = end_points_.begin(); it < end_points_.end(); ) {
    float diff_x = fabs((*it).x() - x);
    float diff_y = fabs((*it).y() - y);
    bool end_point_equal = (diff_x < eps) && (diff_y < eps);
    if (end_point_equal) {
      it = end_points_.erase(it);
      return;
    }
    it++;
  };
}*/

/*
* Returns the end point of a wall. If first is true, the first endpoint
* (north on vertical, west on horizontal) is returned, otherwise
* the second end point is returned (the south on vertical and
* east on horizontal).
*/
/*Eigen::Vector2f Mapping::get_end_point(
  int row,
  int col,
  bool horizontal,
  bool first
)
{
  float x;
  float y;
  if (horizontal) {
    x = row * cell_size_;
    y = (col - 1*(col >= 1 ? 1 : 0)) * cell_size_;
    if  (first) {
      y += cell_size_;
    }
  } else {
    y = col * cell_size_;
    x = (row - 1) * cell_size_;
    if (!first) {
      x += cell_size_;
    }
  };
  Eigen::Vector2f end_point(x, y);
  return end_point;
}*/

/*
 * Converts the second end point (the south end in vertical wall and the east
 * end on horizontal wall) of a wall to it's wall index.
  */
/*int Mapping::convert_to_wall_index(
  Eigen::Vector2f end_point,
  bool horizontal) {
    float x = end_point[0];
    float y = end_point[1];
    assert (cell_size_ != 0);
    int row = round(x / cell_size_);
    int col = round(y / cell_size_);
    if (horizontal && col >= 0) {
      col++;
    }
    int wall_index = get_wall_index(row, col, horizontal);
    return wall_index;
}*/

/* TODO: REMOVE
 * Updates end_points_ and checks if an illegal crossing was made when the
 * horizontal/vertical wall at (row, col) was added. If so it removes the
 * walls at that crossing.
 */
/*void Mapping::secure_wall_validity(int row, int col, bool horizontal) {
  int wall_index = get_wall_index(row, col, horizontal);
  Eigen::Vector2f end_point_1 = get_end_point(row, col, horizontal, true);
  Eigen::Vector2f end_point_2 = get_end_point(row, col, horizontal, false);
  int walls_at_1 = count_connecting_walls_at(end_point_1);
  int walls_at_2 = count_connecting_walls_at(end_point_2);
  ROS_INFO("Nr of walls at 1: %d", walls_at_1);
  ROS_INFO("Nr of walls at 2: %d", walls_at_2);
  if (walls_at_1 >= 3 || walls_at_2 >= 3) {4
    remove_wall_at(row, col, horizontal);
    if (walls_at_1 >= 3) {
      remove_walls_at(end_point_1);
    }
    if (walls_at_2 >= 3){
      remove_walls_at(end_point_2);
    };
  } else {
    update_end_points_old(walls_at_1, end_point_1);
    update_end_points_old(walls_at_2, end_point_2);
  }
}*/

/* TODO: REMOVE
 * void Mapping::update_end_points(Eigen::Vector2f end_point)
 * Takes the number of walls that connects to a crossing, and the position of
 * the crossing. If only one wall connects to this crossings, it's an
 * end_point and is added to end_points_. If there are 2 (it's
 * assumed that no more than 2 walls can be connected), the
 * crossing is removed from end_points_.
 */
/*void Mapping::update_end_points_old(int walls_at, Eigen::Vector2f crossing) {
  if (walls_at == 1) {
    //  Add new end point
    end_points_.push_back(crossing);
  } else {
    // walls_at == 2 and this is no longer an endpoint.
    remove_end_point(crossing);
  }
}*/

/*
 * Removes the north vertical wall from crossing and updates end_points_
 */
/*void Mapping::remove_north_wall(Eigen::Vector2f crossing) {
  Eigen::Vector2f north_end_point = get_north_end_point(crossing);
  int north_wall = convert_to_wall_index(crossing, false);
  remove_wall(north_wall, north_end_point);
}*/


/*
 * Removes the south vertical wall from crossing and updates end_points_
 */
/*void Mapping::remove_south_wall(Eigen::Vector2f crossing) {
  Eigen::Vector2f south_end_point = get_south_end_point(crossing);
  int south_wall = convert_to_wall_index(south_end_point, false);
  remove_wall(south_wall, south_end_point);
}*/


/*
 * Removes the west horizontal wall from crossing and updates end_points_
 */
/*void Mapping::remove_west_wall(Eigen::Vector2f crossing) {
  Eigen::Vector2f west_end_point = get_west_end_point(crossing);
  int west_wall = convert_to_wall_index(crossing, true);
  remove_wall(west_wall, west_end_point);
}*/

/*
 * Removes the east horizontal wall from crossing and updates end_points_
 */
/*void Mapping::remove_east_wall(Eigen::Vector2f crossing) {
  Eigen::Vector2f east_end_point = get_east_end_point(crossing);
  int east_wall = convert_to_wall_index(east_end_point, true);
  remove_wall(east_wall, east_end_point);
}*/

/*
 * Removes wall from walls_ if it exsists and updates end_points_
 */
 /*void Mapping::remove_wall(int wall_index, Eigen::Vector2f end_point) {
   if (wall_exists(wall_index)) {
     walls_[wall_index] = 0;  // remove wall
     update_end_points(end_point);
     }
   }*/

/* TODO: REMOVE
 * Removes wall from walls_ if it exsists and updates end_points_
 */
 /*void Mapping::remove_wall_old(int wall_index, Eigen::Vector2f end_point) {
   if (wall_exists(wall_index)) {
     walls_[wall_index] = 0;  // remove wall
     int nr_of_walls = count_connecting_walls_at(end_point);
     if (nr_of_walls == 0 || nr_of_walls == 2) {
       remove_end_point(end_point);
     } else if (nr_of_walls == 1 || nr_of_walls == 3) {
       // add new end point
       end_points_.push_back(end_point);
     }
  };
}*/

/*
 * Return true if the index is within bounds and a wall exists at this index,
 * otherwise false
 */
/*bool Mapping::wall_exists(int wall_index) {
  if (within_bounds(wall_index)) {
    return walls_[wall_index];
  }
  ROS_INFO("OUTSIDE OF BOUNDS");
  return false;
}*/

/*
 * Remove wall from walls_.
 */
/*void Mapping::remove_wall_at(int row, int col, bool horizontal) {
  if (horizontal) {
    int t = (col >= 1 ? 1 : 0);
    walls_[row*w_ + row*(w_ + 1) + offset_ + col - t] = 0;
  } else {
    walls_[row*w_ + (w_ + 1)*(row - 1) + offset_ + col] = 0;
  };
}*/
