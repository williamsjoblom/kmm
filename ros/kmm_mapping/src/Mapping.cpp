#include "kmm_mapping/Mapping.hpp"

namespace kmm_mapping {

  Mapping::Mapping(ros::NodeHandle nh)
  : nh_(nh)
  {
    // Publishers
    walls_pub_ = nh_.advertise<std_msgs::Int8MultiArray>("walls", 1);
    end_points_pub_ = nh_.advertise<sensor_msgs::PointCloud>("end_points", 1);
    // Subscribers
    sub_ = nh_.subscribe("aligned_scan", 1, &Mapping::mapping_callback, this);

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
    publish_walls_ = false;
    // Set dimensions of wall array in message
    walls_msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
    walls_msg_.layout.dim[0].size = walls_.size();
    walls_msg_.layout.dim[0].stride = 1;
    walls_msg_.layout.dim[0].label = "x";

    // Wall point count requirements
    pnt_cnt_req_ = 7;
    times_req_ = 5;

    // End Points
    publish_end_points_ = false;
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
    if (publish_walls_) {
      publish_walls();
    }
    if (publish_end_points_) {
      publish_end_points();
    }

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
    publish_walls_ = true;
  }

  void Mapping::update_end_points(int row, int col, bool horizontal) {
    // Calculate the two end points associated with wall.
    float x_1;
    float y_1;
    float x_2;
    float y_2;
    if (horizontal) {
      x_1 = row * cell_size_;
      y_1 = (col - 1*(col >= 1 ? 1 : 0)) * cell_size_;
      x_2 = x_1;
      y_2 = y_1 + cell_size_;
    } else {
      x_1 = (row - 1) * cell_size_;
      y_1 = col * cell_size_;
      x_2 = x_1 + cell_size_;
      y_2 = y_1;
    };
    // Determine if end points in list equal any of the two new end points.
    bool found_end_point_1 = false;
    bool found_end_point_2 = false;
    float eps = 0.000001;
    for (auto it = end_points_.begin(); it < end_points_.end(); ) {
      float diff_x_1 = fabs((*it).x() - x_1);
      float diff_y_1 = fabs((*it).y() - y_1);
      bool end_point_1_equal = (diff_x_1 < eps) && (diff_y_1 < eps);
      float diff_x_2 = fabs((*it).x() - x_2);
      float diff_y_2 = fabs((*it).y() - y_2);
      bool end_point_2_equal = (diff_x_2 < eps) && (diff_y_2 < eps);
      // Remove end point from list if we found a match
      if (end_point_1_equal) {
        it = end_points_.erase(it);
        found_end_point_1 = true;
      } else if (end_point_2_equal) {
        it = end_points_.erase(it);
        found_end_point_2 = true;
      } else {
        it++;
      };
    };
    // Add end points to list if we didn't find a match.
    if (!found_end_point_1) {
      Eigen::Vector2f end_point_1;
      end_point_1[0] = x_1;
      end_point_1[1] = y_1;
      end_points_.push_back(end_point_1);
    };
    if (!found_end_point_2) {
      Eigen::Vector2f end_point_2;
      end_point_2[0] = x_2;
      end_point_2[1] = y_2;
      end_points_.push_back(end_point_2);
    };
    publish_end_points_ = true;
  }

  void Mapping::publish_walls() {
    walls_msg_.data.clear();
    for (int& is_wall : walls_) {
        walls_msg_.data.push_back(is_wall);
    }
    walls_pub_.publish(walls_msg_);
    publish_walls_ = false;
  }

  void Mapping::publish_end_points() {
    end_points_msg_.points.clear();
    for (Eigen::Vector2f end_point : end_points_) {
      geometry_msgs::Point32 point;
      point.x = end_point.x();
      point.y = end_point.y();
      point.z = 0;
      end_points_msg_.points.push_back(point);
    };
    end_points_pub_.publish(end_points_msg_);
    publish_end_points_ = false;
  }

  void Mapping::set_pnt_cnt_req(int pnt_cnt_req) {
    pnt_cnt_req_ = pnt_cnt_req;
  }

  void Mapping::set_times_req(int times_req) {
    times_req_ = times_req;
  }
}
