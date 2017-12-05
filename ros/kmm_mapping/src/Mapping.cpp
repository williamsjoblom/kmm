#include "kmm_mapping/Mapping.hpp"

namespace kmm_mapping {

  Mapping::Mapping(ros::NodeHandle nh)
  : nh_(nh)
  {
    // Publishers
    walls_pub_ = nh_.advertise<std_msgs::Int8MultiArray>("walls", 1);
    end_points_pub_ = nh_.advertise<sensor_msgs::PointCloud>("end_points", 1);
    // Subscribers
    aligned_scan_sub_ = nh_.subscribe("aligned_scan", 1, &Mapping::mapping_callback, this);

    // Timers
    publish_walls_timer_ = nh_.createTimer(ros::Duration(1. / 5), &Mapping::publish_walls, this);
    publish_end_points_timer_ = nh_.createTimer(ros::Duration(1. / 5), &Mapping::publish_end_points, this);

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

    // Initialize wall vector with 0's
		for (int i = 0; i < walls_size_; i++) { // 53x103 = (2*height + 1) x (2*width + 1)
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
  }

  Mapping::~Mapping() {
  }

  /* Analyzes points by counting number of collisions on the same wall.
   * Finally publishes the walls that have enough collisions.
   */
  void Mapping::mapping_callback(const sensor_msgs::PointCloud::ConstPtr& msg) {
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
      bool on_horizontal_wall = (rem_x > -eps) && (rem_x < eps);
      float rem_y = std::remainder(std::fabs(y), cell_size_);
      bool on_vertical_wall = (rem_y > -eps) && (rem_y < eps);
      if (on_horizontal_wall) {
        int row = std::round(x / cell_size_);
        int col = std::ceil(std::fabs(y) / cell_size_) * (y < 0 ? -1 : 1); // col can never be 0
        increment_wall_point_count(hor_wall_point_counts_, true, row, col);
      } else if (on_vertical_wall) {
        int row = std::ceil(std::fabs(x) / cell_size_) * (x < 0 ? -1 : 1); // row can never be 0
        int col = std::round(y / cell_size_);
        increment_wall_point_count(ver_wall_point_counts_, false, row, col);
      } else {
        ROS_INFO("Unable to determine wall position! x: %f y: %f fmod x: %f fmod y: %f",
          x, y, std::remainder(std::fabs(x), cell_size_),std::remainder(std::fabs(y), 0.4));
      };
    };
    // Publish
    //publish_walls();
    //publish_end_points();

    reset_wall_point_counts();

    return;
  }

  /* Create new WallPointCount object with given parameters*/
  WallPointCount Mapping::make_wall_point_count(int row, int col, int cnt) {
    Eigen::Vector2f pos(row, col);
    WallPointCount wall_point_count;
    wall_point_count.position = pos;
    wall_point_count.pnt_cnt = cnt;
    wall_point_count.times = 0;
    wall_point_count.published = false;
    return wall_point_count;
  }

  Crossing make_crossing(int x, int y, int count, int wall_index){
    Eigen::Vector2f pos(x, y);
    Crossing crossing;
    crossing.position = pos;
    crossing.nr_of_walls = count;
    crossing.wall_index[0] = wall_index;
    return crossing;
  }


  void Mapping::reset_times(int wall_index) {
    int row;
    int col;
    for (WallPointCount& wall_point_count : hor_wall_point_counts) {
      row = wall_point_count.pos[0];
      col = wall_point_count.pos[1];
      cur_wall_index = get_wall_index(row, col, true);
      if (curr_wall_index == wall_index) {
        wall_point_count.times = 0;
        return;
      }
    };
    for (WallPointCount& wall_point_count :ver_wall_point_counts) {
      row = wall_point_count.pos[0];
      col = wall_point_count.pos[1];
      cur_wall_index = get_wall_index(row, col, false);
      if (curr_wall_index == wall_index) {
        wall_point_count.times = 0;
        return;
      }
    };
  }


  /* Sets all wall point count to 0. Called each time we analyze points. */
  void Mapping::reset_wall_point_counts() {
    // Reset horizontal wall point counts
    for (WallPointCount& wall_point_count : hor_wall_point_counts_) {
      wall_point_count.pnt_cnt = 0;
    };
    // Reset vertical wall point counts
    for (WallPointCount& wall_point_count : ver_wall_point_counts_) {
      wall_point_count.pnt_cnt = 0;
    };
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
      if ((existing_wall_row == row) && (existing_wall_col == col)) {
        wall_point_count.pnt_cnt++;
        if (wall_point_count.pnt_cnt == pnt_cnt_req_ &&
            !wall_point_count.published) {
          wall_point_count.times++;
          if (wall_point_count.times == times_req_) {
            wall_point_count.published = true;
            add_wall(row, col, horizontal);
            update_end_points(row, col, horizontal);
          };
        };
        return;
      };
    };
    wall_point_counts.push_back(make_wall_point_count(row, col, 1));
    return;
  }


  /*
   * Add wall to wall_positions_msg_ and walls_.
   */
  void Mapping::add_wall(int row, int col, bool horizontal) {
    geometry_msgs::Point wall;
    wall.x = row;
    wall.y = col;
    wall.z = 0;
    if (horizontal) {
      int t = (col >= 1 ? 1 : 0);
      walls_[row*w_ + row*(w_ + 1) + offset_ + col - t] = 1;
    } else {
      walls_[row*w_ + (w_ + 1)*(row - 1) + offset_ + col] = 1;
    };
  }


  /*
   * remove wall from walls_. Return true if successfull, otherwise false
   */
  bool Mapping::remove_wall(int row, int col, bool horizontal) {
    int wall_index = get_wall_index(row, col, horizontal);
    if (within_bounds(wall_index)){
      walls_[wall_index] = 0;
      return true;
    }
    return false;
  }

  /* Returns the wall index for wall*/
  int Mapping::get_wall_index(int row, int col, int horizontal) {
    if (horizontal) {
      int t = (col >= 1 ? 1 : 0);
      return row*w_ + row*(w_ + 1) + offset_ + col - t;
    } else {
      return row*w_ + (w_ + 1)*(row - 1) + offset_ + col;
    }
  }

  /*
   * Returns true if the wall index is within bounds.
   */
  bool Mapping::within_bounds(int wall_index) {
    return (wall_index < walls_size_);
  }



  bool Mapping::wall_exists(int row, int col, bool horizontal) {
    int wall_index = get_wall_index(row, col, horizontal);
    return (within_bounds(wall_index) && walls_[wall_index] == 1);
    }


  void Mapping::check_three_way_crossing(int row, int col, bool horizontal) {
    int nr_of_crossings_first;
    int nr_of_crossings_second;
    bool successfull;
    if (horizontal) {
      nr_of_crossings_first = count_crossings_from_horizontal(row, col, true);
      nr_of_crossings_second = count_crossings_from_horizontal(row, col, false);
    } else {
      nr_of_crossings_first = count_crossings_from_vertical(row, col, true);
      nr_of_crossings_second = count_crossings_from_vertical(row, col, false);
    }
    if (nr_of_crossings_first >= 2 && nr_of_crossings_second >= 2) {
      if (horizontal) {
        remove_crossings_from_horizontal(row, col, true);
        remove_crossings_from_horizontal(row, col, false);
      } else {
        remove_crossings_from_vertical(row, col, true);
        remove_crossings_from_vertical(row, col, false);
      }
      remove_end_point(row, col, horizontal, true);
      remove_end_point(row, col, horizontal, false);
      remove_wall(row, col, horizontal);
    } else if (nr_of_crossings_first >= 2) {
      /* Only the first endpoint has illegal amount of crossings, the walls
      that connects to this point is removed. Nothing is done to the second
      end point */
      if (horizontal) {
        remove_crossings_from_horizontal( row, col, true);
      } else {
        remove_crossings_from_vertical(row,  col, true);
      }
      remove_end_point(row, col, horizontal, true);
      remove_wall(row, col, horizontal);
    } else if (nr_of_crossings_second >= 2) {
      /* Only the second endpoint has illegal amount of crossings, the walls
      that connects to this point is removed and nothing is done to the first
      end point */
      if (horizontal) {
        remove_crossings_from_horizontal(row, col, false);
      } else {
        remove_crossings_from_vertical(row, col, false);
      }
      remove_end_point(row, col, horizontal, false);
      remove_wall(row, col, horizontal);
    } else {
      if (nr_of_crossings_first == 0) {
        /* No other wall connects to this side of the wall,
          it's a new end point. */
        add_end_point(row, col, horizontal, true); }
      if (nr_of_crossings_second == 0) {
        /* No other wall connects to this side of the wall,
          it's a new end point. */
        add_end_point(row, col, horizontal, false); }
      if (nr_of_crossings_first == 1){
        /* only one other wall connects to this point, this WAS an end point,
        but it's not any more. Remove this end point */
        bool successfull = remove_end_point(row, col, horizontal, true);
        assert(successfull && "No endpoint was removed!");
      }
      if (nr_of_crossings_second == 1) {
        /* only one other wall connects to this point, this WAS an end point,
        but it's not any more. Remove this end point */
        bool successfull = remove_end_point(row, col, horizontal, false);
        assert(successfull && "No endpoint was removed!");
      }
    }
}


/*
  Removes the walls connected to the wall at (row, col) on the first or
  second side. Updates the end points.
*/
  int Mapping::remove_crossings_from_horizontal(int row, int col, bool first){
    int col_adjustment;
    if (first) {
      // Removing the left side of the hotizontal wall (  _ | _    )
      //                                               (    |      )
      col_adjustment = (col < 0 ? 1 : 0);
      // Removing the wall to the left ( _ _  )
      if (remove_wall(row, col + 1, true)){
        if (!remove_end_point(row, col, true, true)){
          add_end_point(row, col, true, true);
        }
      }
      // Removing the upper vertical wall to to the left (  |_ )
      if (remove_wall(row + 1, col + col_adjustment, false )){
        if (!remove_end_point(row + 1, col + col_adjustment, false, true)){
          add_end_point(row + 1, col + col_adjustment, false, true);
        }
      }
      // Removing the lower vertical wall to the left (   _  )
      //                                              (  |   )
      if (remove_wall(row, col + col_adjustment, false)){
        if (!remove_end_point(row, col + col_adjustment, false, false)){
          add_end_point(row, col + col_adjustment, false, false);
        }
      }
    } else {
      // Removing the right side of the hotizontal wall (    _ | _  )
      //                                                (      |    )
      col_adjustment = (col < 0 ? 0 : - 1);
        // Removing the wall to the right (  _ _ )
      if (remove_wall(row, col - 1, true)) {
        if (!remove_end_point(row, col - 1 , true, false )) {
          add_end_point(row, col - 1 , true, false);
        }
      }
      // Removing the upper vertical wall to to the right ( _|  )
      if (remove_wall(row + 1, col + col_adjustment, false)) {
        if (!remove_end_point(row + 1, col + col_adjustment, false, true)) {
          add_end_point(row + 1, col + col_adjustment, false, true);
        }
      }
      // Removing the lower vertical wall to the right (   _  )
      //                                              (     | )
      if (remove_wall(row, col + col_adjustment, false)) {
        if (!remove_end_point(row, col + col_adjustment, false, false)) {
          add_end_point(row, col + col_adjustment, false, false);
        }
      }
    }
  }

/*
  Counts the amount of endpoints on either the first or second endpoint of
  the horizontal wall on (col, row), in walls_
*/
  int Mapping::count_crossings_from_horizontal(int row, int col, bool first){
    int count = 0;
    int col_adjustment;
    if (first) {
      // Checking the left side of the hotizontal wall (  _ | _    )
      //                                               (    |      )
      col_adjustment = (col < 0 ? 1 : 0);
      // checking the wall to the left ( _ _  )
      if (wall_exists(row, col + 1, true)) { //
        count++;
      }
      // checking the upper vertical wall to to the left (  |_ )
      if (wall_exists(row + 1, col + col_adjustment, false )) {
        count++;
      }
      // checking the lower vertical wall to the left (   _  )
      //                                              (  |   )
      if (wall_exists(row, col + col_adjustment, false)){
        count++;
      }
    } else {
      // Checking the right side of the hotizontal wall (    _ | _  )
      //                                                (      |    )
      col_adjustment = (col < 0 ? 0 : - 1);
      // checking the wall to the right (  _ _ )
      if (wall_exists(row, col - 1, true)) { //
        count++;
      }
      // checking the upper vertical wall to to the right ( _|  )
      if (wall_exists(row + 1, col + col_adjustment, false )) {
        count++;
      }
      // checking the lower vertical wall to the right (   _  )
      //                                              (     | )
      if (wall_exists(row, col + col_adjustment, false)) {
        count++;
      }
    }
    return count;
  }

  int Mapping::count_crossings_from_vertical(int row, int col, bool first){
    int count = 0;
    int col_adjustment;
    if (first) {
      // Checking the lower side of the vertical wall  (  _ | _  )
      //                                               (    |    )
      col_adjustment = (col < 0 ? -1 : 1);
      // checking the wall to the lower left ( _ | ) if col > 0 otherwise the
      // wall to the lower right ( |_ )
      if (wall_exists(row-1, col+col_adjustment, true)) {
        count++;
      }
      // checking the wall to the lower right (  |_ ) if col > 0 otherwise the
      // wall to the lower left ( _| )
      if (wall_exists(row-1, col, true)) {
        count++;
      }
      // checking the vertical wall under  (  |  )
      //                                   (  |  )
      if (wall_exists(row-1, col, false)) {
        count++;
      }
    } else {
      // Checking the upper side of the vertical wall (  _ | _  )
      //                                              (    |    )
      col_adjustment = (col < 0 ? -1 : 1);
      // checking the horizontal wall to the upper left if col > 0 otherwise
      // the one to the upper right
      if (wall_exists(row, col+col_adjustment, true)) {
        count++;
      }
      // checking the horizontal wall to the upper right if col > 0
      // otherwise the wall to the upper left
      if (wall_exists(row, col, true)) {
        count++;
      }
      // checking the vertical wall over  (  |  )
      //                                  (  |  )
      if (wall_exists(row+1, col, false)) {
        count++;
      }
    }
    return count;
  }


  int Mapping::remove_crossings_from_vertical(int row, int col, bool first){
    int col_adjustment;
    bool is_first = (col > 0);
    if (first) {
      // removing the lower side of the vertical wall  (  _ | _  )
      //                                               (    |    )
      col_adjustment = (col < 0 ? -1 : 1);
      // removing the wall to the lower left ( _ | ) if col > 0 otherwise the
      // wall to the lower right ( |_ )
      if (remove_wall(row-1, col+col_adjustment, true)) {
        if (!remove_end_point(row-1, col+col_adjustment, true, is_first)) {
          add_end_point(row-1, col+col_adjustment, true, is_first);
        }
      }
      // removing the wall to the lower right (  |_ ) if col > 0 otherwise the
      // wall to the lower left ( _| )
      if (remove_wall(row-1, col, true)) {
        if (!remove_end_point(row-1, col, true, !is_first)) {
          add_end_point(row-1, col, true, !is_first);
        }
      }
      // removing the vertical wall under  (  |  )
      //                                   (  |  )
      if (remove_wall(row-1, col, false)) {
        if (!remove_end_point(row-1, col, false, false)) {
          add_end_point(row-1, col, false, false);
        }
      }
    } else {
        // Removing the upper side of the vertical wall (  _ | _  )
        //                                              (    |    )
        col_adjustment = (col < 0 ? -1 : 1);
        // removing the horizontal wall to the upper left if col > 0 otherwise
        // the one to the upper right
        if (remove_wall(row, col+col_adjustment, true)) {
          if (!remove_end_point(row, col+col_adjustment, true, is_first)) {
            add_end_point(row, col+col_adjustment, true, is_first);
          }
        }
        // removing the horizontal wall to the upper right if col > 0
        // otherwise the wall to the upper left
        if (remove_wall(row, col, true)) {
          if (!remove_end_point(row, col, true, !is_first)){
            add_end_point(row, col, true, !is_first);
          }
        }
        // Removing the vertical wall over  (  |  )
        //                                  (  |  )
        if (remove_wall(row+1, col, false)) {
          if (!remove_end_point(row+1, col, false, true)) {
            add_end_point(row+1, col, false, true);
          }
        }
    }
  }





  void Mapping::get_end_points(
    int row,
    int col,
    bool horizontal,
    std::pair<float,float> end_points[2]
  )
  {
    float x_1;
    float y_1;
    float x_2;
    float y_2;
    if (horizontal) {
      x_2 = row * cell_size_;
      y_2 = (col - 1*(col >= 1 ? 1 : 0)) * cell_size_;
      x_1 = x_2;
      y_1 = y_2 + cell_size_;
    } else {
      x_1 = (row - 1) * cell_size_;
      y_1 = col * cell_size_;
      x_2 = x_1 + cell_size_;
      y_2 = y_1;
    };
    std::pair<float, float> first_end_point(x_1, y_1);
    std::pair<float, float> second_end_point(x_2, y_2);
    end_points[0] = first_end_point;
    end_points[1] = second_end_point;
  }


  Eigen::Vector2f Mapping::get_end_point(
    int row,
    int col,
    bool horizontal,
    bool first
  )
  {
    float x_1;
    float y_1;
    float x_2;
    float y_2;
    if (horizontal) {
      x_2 = row * cell_size_;
      y_2 = (col - 1*(col >= 1 ? 1 : 0)) * cell_size_;
      x_1 = x_2;
      y_1 = y_2 + cell_size_;
    } else {
      x_1 = (row - 1) * cell_size_;
      y_1 = col * cell_size_;
      x_2 = x_1 + cell_size_;
      y_2 = y_1;
    };
    Eigen::Vector2f end_point;
    if (first){
      end_point[0] = x_1;
      end_point[1] = y_1;
    } else {
      end_point[0] = x_2;
      end_point[1] = y_2;
    }
    return end_point;
  }


  Eigen::Vector2f Mapping::remove_wallsdfsasdf(
    int row,
    int col,
    bool horizontal,
    Crossing crossing,
    int original_wall
  )
  {
    Eigen::Vector2f start_point = crossing.position;
    Eigen::Vector2f upper(start_point[0] - cell_size_, start_point[1]);
    Eigen::Vector2f lower(start_point[0] + cell_size_, start_point[1]);
    Eigen::Vector2f left(start_point[0], start_point[1] + cell_size_);
    Eigen::Vector2f right(start_point[0], start_point[1] - cell_size_);

    upper_wall = translate_to_wall_index(start_point, false); //upper wall
    lower_wall = translate_to_wall_index(lower, false);       // lower wall
    right_wall = translate_to_wall_index(right, true);        // right wall
    left_wall = translate_to_wall_index(start_point, true);   // left wall
    if (upper_wall == crossing.wall_index[0] ||
        upper_wall == crossing.wall_index[1]) {
        //TODO: find end point in list, count down
    }
    if (lower_wall == crossing.wall_index[0]  ||
        lower_wall == crossing.wall_index[1]) {
        //TODO: find end point in list, count down
    }
    if (right_wall == crossing.wall_index[0]  ||
        right_wall == crossing.wall_index[1]) {
        //TODO: find end point in list, count down
    }
    if (left_wall == crossing.wall_index[0] ||
        left_wall == crossing.wall_index[1]) {
        //TODO: find end point in list, count down
    }


  }

  void mapping::count_down_end_point(Eigen::Vector2f end_point, int wall_index) {
    Eigen::Vector2f curr_end_point;
    for (auto it = end_points_.begin(); it < end_points_.end(); ) {
      curr_end_point = (*it).position;
      if (is_equal_to(end_point, curr_end_point)) {
        (*it).nr_of_walls--;
        int nr_of_walls = (*it).nr_of_walls;
        if (nr_of_walls == 0) {
          it = end_points_.erase(it);
        } else {
          for (int i = 0; i < nr_of_walls; i++ ) {
            if ((*it).nr_of_walls[i] == wall_index) {
              (*it).nr_of_walls.erase()
          }
        }
      }
  }



/* translates the second end point (the lower end in vertical wall and right
    side on horizontal wall) of a wall to it's wall index.
*/
int Mapping::translate_to_wall_index(Eigen::Vector2f end_point,
  bool horizontal) {
    float x = end_point[0];
    float y = end_point[1];
    int row = round(x / cell_size_);
    int col = round(y / cell_size_);
    if (horizontal && col >= 1) {
      col++;
    }
    int wall_index = get_wall_index(row, col, horizontal);
    return wall_index;
  }



/* returns true if the end point was removed, otherwise false, "first" stands for
  if its the first or second endpoint on the wall*/
  bool Mapping::remove_end_point(int row, int col, bool horizontal, bool first){

    Eigen::Vector2f end_point = get_end_point(row, col, horizontal, first);
    float x = end_point[0];
    float y = end_point[1];

    float eps = 0.000001;
    for (auto it = end_points_.begin(); it < end_points_.end(); ) {
      float diff_x = fabs((*it).x() - x);
      float diff_y = fabs((*it).y() - y);
      bool end_point_equal = (diff_x < eps) && (diff_y < eps);
      // Remove end point from list if we found a match
      if (end_point_equal) {
        end_points_.erase(it);
        return true;
      } else {
        it++;
      }
    }
    return false;
  }

  void Mapping::add_end_point(int row, int col, bool horizontal, bool first){
    std::pair<float, float> end_points[2];
    // Saves the coordinates for the first end point (x1, y1) and second
    // end point (x2, y2) in end_points.
    float x;
    float y;
    get_end_points(row, col, horizontal, end_points);
    if (first) {
      x = end_points[0].first;
      y = end_points[0].second;
    } else {
      x = end_points[1].first;
      y = end_points[1].second;
    }
    Eigen::Vector2f end_point;
    end_point[0] = x;
    end_point[1] = y;
    end_points_.push_back(end_point);
  }

  void Mapping::update_end_points(int row, int col, bool horizontal) {
    Eigen::Vector2f end_point = get_end_point(row, col, horizontal, true);
    float x_1 = end_point[0];
    float y_1 = end_point[1];
    Eigen::Vector2f end_point = get_end_point(row, col, horizontal, false);
    float x_2 = end_point[0];
    float y_2 = end_point[1];
    Crossing end_point_1;
    Crossing end_point_2;
    bool found_end_point_1 = false;
    bool found_end_point_2 = false;
    float eps = 0.000001;
    for (auto it = end_points_.begin(); it < end_points_.end(); ) {
      float diff_x_1 = fabs((*it).position.x() - x_1);
      float diff_y_1 = fabs((*it).position.y() - y_1);
      bool end_point_equal_1 = (diff_x_1 < eps) && (diff_y_1 < eps);
      float diff_x_2 = fabs((*it).position.x() - x_2);
      float diff_y_2 = fabs((*it).position.y() - y_2);
      bool end_point_equal_2 = (diff_x_2 < eps) && (diff_y_2 < eps);
      // Remove end point from list if we found a match
      if (end_point_equal_1) {
        end_point_1 = *it;      //TODO: Check if this is ok!
        it = end_points_.erase(it);
      } else if (end_point_equal_2) {
        end_point_2 = *it;
        it = end_points_.erase(it);
      } else {
          it++;
      }
    }
    if (found_end_point_1 && end_point_1.nr_of_walls >= 2) {
      //remove walls_
      // update endpoints
    }
    return false;
  }


/* Return true if one == two, otherwise false*/
  bool Mapping::is_equal_to(Eigen::Vector2f one, Eigen::Vector2f two){
    float eps = 0.000001;
    float diff_x = fabs(one.x() - two.x());
    float diff_y = fabs(one.y() - two.y());
    bool point_equal = (diff_x < eps) && (diff_y < eps);
    return point_equal;
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

  void Mapping::set_pnt_cnt_req(int pnt_cnt_req) {
    pnt_cnt_req_ = pnt_cnt_req;
  }

  void Mapping::set_times_req(int times_req) {
    times_req_ = times_req;
  }
}
