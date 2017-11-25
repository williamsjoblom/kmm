#pragma once
#include <Eigen/Dense>


namespace kmm_navigation {

  class PathFollower {
  public:
    PathFollower();
    Eigen::Vector2f get_velocity(std::vector<Eigen::Vector2f> path, Eigen::Vector2f robot_position);
  };
}
