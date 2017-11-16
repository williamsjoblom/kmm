#include "kmm_mapping/Mapping.hpp"
#include "geometry_msgs/Point.h"

namespace kmm_mapping {

  Mapping::Mapping(ros::NodeHandle nh)
  : nh_(nh)
  {
    // Publishers
    mapping_pub_ = nh_.advertise<kmm_mapping::wall_positions>("wall_positions", 1);
    // Subscribers
    mapping_sub_ = nh_.subscribe("aligned_scan", 1, &Mapping::mapping_callback, this);
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

    /* Increment wall point counts. Will add walls that haven't been added before. */
    for (int i = 0; i < wall_points.size(); i++) {
      int row;
      int col;
      float x = wall_points[i].x();
      float y = wall_points[i].y();
      float eps = 0.000001;
      float rem_x = std::remainder(std::fabs(x), 0.4);
      bool on_horizontal_wall = (rem_x > -eps) && (rem_x < eps);
      float rem_y = std::remainder(std::fabs(y), 0.4);
      bool on_vertical_wall = (rem_y > -eps) && (rem_y < eps);
      if (on_horizontal_wall) { // point on horizontal wall
        row = std::round(x / 0.4); // integer, not float
        col = std::ceil(std::fabs(y) / 0.4) * (y < 0 ? -1 : 1); // can never be 0
        hor_wall_point_counts_ = increment_wall_point_count(hor_wall_point_counts_, true, row, col);
      } else if (on_vertical_wall) { // point on vertical wall
        row = std::ceil(std::fabs(x) / 0.4) * (x < 0 ? -1 : 1); // can never be 0
        col = std::round(y / 0.4); // integer, not float
        ver_wall_point_counts_ = increment_wall_point_count(ver_wall_point_counts_, false, row, col);
      } else {
        ROS_INFO("Point from /aligned_scan missing x or y as multiple of 0.4! x: %f y: %f fmod x: %f fmod y: %f",
          x, y, std::remainder(std::fabs(x), 0.4),std::remainder(std::fabs(y), 0.4));
      };
    };

    wall_positions_msg_.cnt = msg_cnt_;
    ROS_INFO("wall_positions_node publishing message %d", wall_positions_msg_.cnt);
    mapping_pub_.publish(wall_positions_msg_);
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
  std::vector<WallPointCount> Mapping::increment_wall_point_count(
      std::vector<WallPointCount> wall_point_counts,
      bool horizontal, int row, int col) {
    for (std::vector<WallPointCount>::iterator it = wall_point_counts.begin();
    it != wall_point_counts.end(); it++) {
      if (((*it).position[0] == row) && ((*it).position[1] == col)) {
        (*it).pnt_cnt++;
        if ((*it).pnt_cnt == 7 and !(*it).published) {
          (*it).times++;
          if ((*it).times == 5) {
            (*it).published = true;
            geometry_msgs::Point wall;
            wall.x = (*it).position[0];
            wall.y = (*it).position[1];
            wall.z = 0;
            if (horizontal) {
              ROS_INFO("Pushed back horizontal!");
              wall_positions_msg_.horizontal_walls.push_back(wall);
            } else {
              ROS_INFO("Pushed back vertical!");
              wall_positions_msg_.vertical_walls.push_back(wall);
            };
          };
        };
        // We found the wall point count for (row,col)
        return wall_point_counts;
      };
    };
    // We have to create new wall point count for (row,col). Set pnt_cnt to 1.
    wall_point_counts.push_back(make_wall_point_count(row, col, 1));
    return wall_point_counts;
  }

  // Returns random values uniformly distributed in the range [a, b]
  int Mapping::random(const int a, const int b) {
    thread_local std::mt19937 eng{std::random_device{}()};
    std::uniform_int_distribution<int> dist(a, b);
    return dist(eng);
  }
}


  //ros::Rate loop_rate(10);

  /*while (ros::ok())
  {
    wall_positions_msg.horizontal_walls.clear();
    wall_positions_msg.vertical_walls.clear();
    wall_positions_msg.cnt = count;

    for (int i = 0; i < 25; i++) {
        geometry_msgs::Point point_horizontal;
        point_horizontal.x = random(0,25);
        point_horizontal.y = random(-26,26); // can never be 0
        while (point_horizontal.y == 0) { // make sure y != 0
          point_horizontal.y = random(-26,26);
        }
        point_horizontal.z = 0;
        wall_positions_msg.horizontal_walls.push_back(point_horizontal);
    }

    for (int i = 0; i < 17; i++) {
        geometry_msgs::Point point_vertical;
        point_vertical.x = random(1,25); // can never be 0
        point_vertical.y = random(-26,26);
        point_vertical.z = 0;
        wall_positions_msg.vertical_walls.push_back(point_vertical);
    }

    ROS_INFO("%d", wall_positions_msg.cnt);

    pub.publish(wall_positions_msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }*/

  /*
  // Add horizontal walls
  for (std::vector<WallPointCount>::iterator it = horizontal_wall_point_counts.begin(),
      horizontal_wall_point_counts.end(); it != horizontal_wall_point_counts.end()) {
    if ((*it).count > 5) {
      geometry_msgs::Point horizontal_wall;
      horizontal_wall.x = (*it).position.x;
      horizontal_wall.y = (*it).position.y;
      horizontal_wall.z = 0;
      wall_positions_msg.horizontal_walls.push_back(horizontal_wall);
    };
  };
  // Add vertical walls
  for (std::vector<WallPointCount>::iterator it = vertical_wall_point_counts.begin(),
      vertical_wall_point_counts.end(); it != vertical_wall_point_counts.end()) {
    if ((*it).count > 5) {
      geometry_msgs::Point vertical_wall;
      vertical_wall.x = (*it).position.x;
      vertical_wall.y = (*it).position.y;
      vertical_wall.z = 0;
      wall_positions_msg.vertical_walls.push_back(vertical_wall);
    };
  };*/
