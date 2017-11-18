#include "kmm_mapping/Mapping.hpp"
#include "geometry_msgs/Point.h"

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

    // Initialize wall array with 0's.
		wall_arr_msg_.data.clear();
		for (int i = 0; i < 5459; i++) { // 53x103 = (2*height + 1) x (2*width + 1)
			wall_arr_msg_.data.push_back(0);
    };

    w_ = 51;
    offset_ = (w_ - 1) / 2;
  }

  Mapping::~Mapping() {
  }

  /* Analyzes points by counting number of collisions on the same wall.
   * Finally publishes the walls that have enough collisions.
   */
  void Mapping::mapping_callback(const sensor_msgs::PointCloud::ConstPtr& msg) {
    ROS_INFO("wall_positions_node recieved a message from /aligned_scan");
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
    wall_positions_msg_.cnt = msg_cnt_;
    ROS_INFO("wall_positions_node publishing message %d", wall_positions_msg_.cnt);
    mapping_wall_pos_pub_.publish(wall_positions_msg_);
    mapping_wall_arr_pub_.publish(wall_arr_msg_);
    end_points_msg_.points.clear();
    for (int i = 0; i < end_points_.size(); i++) {
      geometry_msgs::Point32 point;
      point.x = end_points_[i].x();
      point.y = end_points_[i].y();
      point.z = 0;
      end_points_msg_.points.push_back(point);
    };
    mapping_end_points_pub_.publish(end_points_msg_);
    ++msg_cnt_;
    reset_wall_point_counts();

    return;
  }

  /* Create new WallPointCount object with given parameters*/
  WallPointCount Mapping::make_wall_point_count(int row, int col, int cnt) {
    Eigen::Vector2f pos;
    pos[0] = row;
    pos[1] = col;
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
            geometry_msgs::Point wall;
            wall.x = row;
            wall.y = col;
            wall.z = 0;
            if (horizontal) {
              ROS_INFO("Found horizontal wall!");
              wall_positions_msg_.horizontal_walls.push_back(wall);
              wall_arr_msg_.data[row*w_ + (w_ + 1)*row + offset_ + col] = 1;
              update_end_points(row, col, true);
            } else {
              ROS_INFO("Found vertical wall!");
              wall_positions_msg_.vertical_walls.push_back(wall);
              wall_arr_msg_.data[row*w_ + (w_ + 1)*(row - 1) + offset_ + col + 1] = 1;
              update_end_points(row, col, false);
            };
          };
        };
        return;
      };
    };
    // We have to create new wall point count for (row,col). Set pnt_cnt to 1.
    wall_point_counts.push_back(make_wall_point_count(row, col, 1));
    return;
  }

  void Mapping::update_end_points(int row, int col, bool horizontal) {
    float x_1;
    float y_1;
    float x_2;
    float y_2;
    if (horizontal) {
      x_1 = row * 0.4;
      y_1 = (col - 1*(col >= 1 ? 1 : 0)) * 0.4;

      x_2 = row * 0.4;
      y_2 = y_1 + 0.4;
    } else {
      x_1 = (row - 1) * 0.4;
      y_1 = col * 0.4;

      x_2 = row * 0.4;
      y_2 = col * 0.4;
    };

    bool found_end_point_1 = false;
    bool found_end_point_2 = false;
    for (auto it = end_points_.begin(); it != end_points_.end(); ) {
      if ((*it).x() == x_1 && (*it).y() == y_1) {
        it = end_points_.erase(it++);
        found_end_point_1 = true;
      } else if ((*it).x() == x_2 && (*it).y() == y_2) {
        it = end_points_.erase(it++);
        found_end_point_2 = true;
      } else {
        ++it;
      };
    };
    if (!found_end_point_1) {
      Eigen::Vector2f end_point;
      end_point[0] = x_1;
      end_point[1] = y_1;
      end_points_.push_back(end_point);
    };
    if (!found_end_point_2) {
      Eigen::Vector2f end_point;
      end_point[0] = x_2;
      end_point[1] = y_2;
      end_points_.push_back(end_point);
    };
  }
}
