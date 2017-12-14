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
    auto_mode_sub_ = nh_.subscribe("auto_mode", 1, &Mapping::auto_mode_callback, this);

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
        assert(false);
    }
    if (!nh_.getParam("/map_cols", w_)) {
        ROS_ERROR("Couldn't set map_cols!");
        assert(false);
    }
    if (!nh_.getParam("/cell_size", cell_size_)) {
        ROS_ERROR("Couldn't set cell_size!");
        assert(false);
    }
    offset_ = (w_ - 1) / 2;
    walls_size_ = (w_ + (w_ + 1)) * h_ + w_;

    // Set initial values
    mapping_ = true;
    auto_mode_ = false;

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


  /*
   * SUBSCRIBER CALLBACKS
   */

   void Mapping::auto_mode_callback(const std_msgs::Bool::ConstPtr& msg) {
     auto_mode_ = msg->data;
     if (auto_mode_) {
       mapping_ = true;
     }
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
    int index;
    if (horizontal) {
      int t = (col >= 1 ? 1 : 0);
      index  = row*w_ + row*(w_ + 1) + offset_ + col - t;
    } else {
      index = row*w_ + (w_ + 1)*(row - 1) + offset_ + col;
    }
    if (is_wall_index_within_bounds(index)) {
      walls_[index] = 1;
    }
  }

  /*
   * Action server callback. Gets an end point that kmm_exploration has
   * deemed unreachable. Removes it and all walls connected to it.
  */
  void Mapping::remove_walls_callback(const kmm_mapping::RemoveWallsGoalConstPtr &end_point) {
    Eigen::Vector2f e(end_point->x, end_point->y);
    remove_walls_at_crossing(e);
    toggle_end_point(e);
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
    int cs_mults_x = crossing.x() / cell_size_; // cell_size multiples
    int cs_mults_y = crossing.y() / cell_size_; // cell_size multiples

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
    int cs_mults_x = crossing.x() / cell_size_; // cell_size multiples
    int cs_mults_y = crossing.y() / cell_size_; // cell_size multiples

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
    int cs_mults_x = crossing.x() / cell_size_; // cell_size multiples
    int cs_mults_y = crossing.y() / cell_size_; // cell_size multiples

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
    int cs_mults_x = crossing.x() / cell_size_; // cell_size multiples
    int cs_mults_y = crossing.y() / cell_size_; // cell_size multiples

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
      if (col == 0 || row < 0) { // Illegal row for horizontal
        return false;
      };
      int t = (col >= 1 ? 1 : 0);
      return walls_[row*w_ + row*(w_ + 1) + offset_ + col - t];
    } else {
      if (row == 0) { // Illegal row for vertical
        return false;
      };
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
    bool end_point_x_within_bounds = end_point.x() >= 0 && end_point.x() <= h_ * cell_size_;
    float abs_y_limit = cell_size_ * offset_; // offset = (map_width - 1) / 2
    bool end_point_y_within_bounds = end_point.y() >= -abs_y_limit
      && end_point.y() <= abs_y_limit + cell_size_; // positive side has one extra multiple
    bool end_point_within_bounds = end_point_x_within_bounds
      && end_point_y_within_bounds;
    if (end_point_within_bounds) {
      for (auto it = end_points_.begin(); it < end_points_.end(); ) {
        if (are_equal(end_point, *it)) {
          it = end_points_.erase(it);
          return;
        }
        it++;
      };
      end_points_.push_back(end_point);
    }
  }

  /*
   * Returns the end point north of crossing
   */
  Eigen::Vector2f Mapping::get_north_end_point(Eigen::Vector2f crossing) {
    Eigen::Vector2f north(crossing.x() + cell_size_, crossing.y());
    return north;
  }

  /*
   * Returns the end point south of crossing
   */
  Eigen::Vector2f Mapping::get_south_end_point(Eigen::Vector2f crossing) {
    Eigen::Vector2f south(crossing.x() - cell_size_, crossing.y());
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
    if (!auto_mode_) { // mapping can't be toggled in auto mode
      mapping_ = req.data;
    }
    return true;
  }

  /*
   * Callback for service requests to reset map.
   */
  bool Mapping::reset_map(std_srvs::SetBool::Request &req,
    std_srvs::SetBool::Response &res) {
    if (!auto_mode_) {
      // Reset walls
      walls_.clear();
      for (int i = 0; i < walls_size_; i++) {
  			walls_.push_back(0);
      };

      // Reset wall point counts
      hor_wall_point_counts_.clear();
      ver_wall_point_counts_.clear();

      end_points_.clear();
    }

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
