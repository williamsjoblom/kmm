#pragma once
#include <Eigen/Dense>


namespace kmm_navigation {

  class PathFollower {
  public:
    PathFollower();
    void get_velocity(
      const std::vector<Eigen::Vector2f>& path,
      const Eigen::Vector2f& robot_position,
      Eigen::Vector2f& vel,
      bool& has_reached_target);
  };
}
