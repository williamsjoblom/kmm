#include "kmm_mapping/Mapping.hpp"

namespace kmm_mapping {

  Mapping::Mapping(ros::NodeHandle nh)
  : nh_(nh)
  {
    // Publishers
    mapping_wall_pos_pub_ = nh_.advertise<kmm_mapping::wall_positions>("wall_positions", 1);
    mapping_wall_arr_pub_ = nh_.advertise<std_msgs::Int8MultiArray>("wall_array", 1);
    mapping_end_points_pub_ = nh_.advertise<sensor_msgs::PointCloud>("end_points", 1);
    // Subscribers
    mapping_sub_ = nh_.subscribe("aligned_scan", 1, &Mapping::mapping_callback, this);

    // Initialize wall vector with 0's
		for (int i = 0; i < 1500; i++) { // 53x103 = (2*height + 1) x (2*width + 1)
			wall_vec_.push_back(0);
    };
    // Set dimensions of wall array in message
    wall_arr_msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
    wall_arr_msg_.layout.dim[0].size = wall_vec_.size();
    wall_arr_msg_.layout.dim[0].stride = 1;
    wall_arr_msg_.layout.dim[0].label = "x";
  }

  Mapping::~Mapping() {
  }

  /* Analyzes points by counting number of collisions on the same wall.
   * Finally publishes the walls that have enough collisions.
   */
  void Mapping::mapping_callback(const sensor_msgs::PointCloud::ConstPtr& msg) {
    std::vector<Eigen::Vector2f> wall_points;
    for (const auto p : msg->points) {
      Eigen::Vector2f point;
      point[0] = p.x;
      point[1] = p.y;
      wall_points.push_back(point);
    };
    // Iterate through wall points.
    for (int i = 0; i < wall_points.size(); i++) {
      float x = wall_points[i].x();
      float y = wall_points[i].y();
      float eps = 0.000001;
      float rem_x = std::remainder(std::fabs(x), 0.4);
      bool on_horizontal_wall = (rem_x > -eps) && (rem_x < eps);
      float rem_y = std::remainder(std::fabs(y), 0.4);
      bool on_vertical_wall = (rem_y > -eps) && (rem_y < eps);
      if (on_horizontal_wall) {
        int row = std::round(x / 0.4);
        int col = std::ceil(std::fabs(y) / 0.4) * (y < 0 ? -1 : 1); // col can never be 0
        increment_wall_point_count(hor_wall_point_counts_, true, row, col);
      } else if (on_vertical_wall) {
        int row = std::ceil(std::fabs(x) / 0.4) * (x < 0 ? -1 : 1); // row can never be 0
        int col = std::round(y / 0.4);
        increment_wall_point_count(ver_wall_point_counts_, false, row, col);
      } else {
        ROS_INFO("Unable to determine wall position! x: %f y: %f fmod x: %f fmod y: %f",
          x, y, std::remainder(std::fabs(x), 0.4),std::remainder(std::fabs(y), 0.4));
      };
    };
    // Publish
    publish_wall_positions();
    publish_wall_array();
    publish_end_points();

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
    for (std::vector<WallPointCount>::iterator it = hor_wall_point_counts_.begin();
        it != hor_wall_point_counts_.end(); it++) {
      (*it).pnt_cnt = 0;
    };
    // Reset vertical wall point counts
    for (std::vector<WallPointCount>::iterator it = ver_wall_point_counts_.begin();
        it != ver_wall_point_counts_.end(); it++) {
      (*it).pnt_cnt = 0;
    };
  }

  /* Increment the wall point count of a wall. Creates a new one with count 1 if missing. */
  void Mapping::increment_wall_point_count(
      std::vector<WallPointCount>& wall_point_counts,
      bool horizontal, int row, int col) {
    int existing_wall_row;
    int existing_wall_col;
    for (std::vector<WallPointCount>::iterator it = wall_point_counts.begin();
        it != wall_point_counts.end(); it++) {
      existing_wall_row = (*it).position[0];
      existing_wall_col = (*it).position[1];
      if ((existing_wall_row == row) && (existing_wall_col == col)) {
        (*it).pnt_cnt++;
        if ((*it).pnt_cnt == pnt_cnt_req_ && !(*it).published) {
          (*it).times++;
          if ((*it).times == times_req_) {
            (*it).published = true;
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

  void Mapping::add_wall(int row, int col, bool horizontal) {
    int w = 53; // Grid width
    int offset = (w - 1) / 2;
    geometry_msgs::Point wall;
    wall.x = row;
    wall.y = col;
    wall.z = 0;
    if (horizontal) {
      wall_positions_msg_.horizontal_walls.push_back(wall);
      int t = (col >= 1 ? 1 : 0);
      wall_vec_[row*w + row*(w + 1) + offset + col - t] = 1;
    } else {
      wall_positions_msg_.vertical_walls.push_back(wall);
      wall_vec_[row*w + (w + 1)*(row - 1) + offset + col] = 1;
    };
  }

  void Mapping::update_end_points(int row, int col, bool horizontal) {
    // Calculate the two end points associated with wall.
    float x_1;
    float y_1;
    float x_2;
    float y_2;
    if (horizontal) {
      x_1 = row * 0.4;
      y_1 = (col - 1*(col >= 1 ? 1 : 0)) * 0.4;
      x_2 = x_1;
      y_2 = y_1 + 0.4;
    } else {
      x_1 = (row - 1) * 0.4;
      y_1 = col * 0.4;
      x_2 = x_1 + 0.4;
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
  }

  void Mapping::publish_wall_positions() {
    wall_positions_msg_.cnt = msg_cnt_;
    mapping_wall_pos_pub_.publish(wall_positions_msg_);
    ++msg_cnt_;
  }

  void Mapping::publish_wall_array() {
    wall_arr_msg_.data.clear();
    for (std::vector<int>::const_iterator it = wall_vec_.begin(); it != wall_vec_.end(); ++it) {
        wall_arr_msg_.data.push_back(*it);
    }
    mapping_wall_arr_pub_.publish(wall_arr_msg_);
  }

  void Mapping::publish_end_points() {
    end_points_msg_.points.clear();
    for (int i = 0; i < end_points_.size(); i++) {
      geometry_msgs::Point32 point;
      point.x = end_points_[i].x();
      point.y = end_points_[i].y();
      point.z = 0;
      end_points_msg_.points.push_back(point);
    };
    mapping_end_points_pub_.publish(end_points_msg_);
  }
}
