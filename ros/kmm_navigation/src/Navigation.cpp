#include "kmm_navigation/Navigation.hpp"

namespace kmm_navigation {

  Navigation::Navigation(ros::NodeHandle nh)
  : nh_(nh), map_(53) {
    // Publishers

    // Subscribers
    wall_array_sub_ = nh_.subscribe("wall_array", 1, &Navigation::wall_array_callback, this);
    position_sub_ = nh_.subscribe("position", 1, &Navigation::position_callback, this);
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
}
