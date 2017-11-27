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
      void set_error_p_constant(float error_p_constant);
      void set_max_velocity(float max_velocity);
  private:
    float error_p_constant_;
    float max_velocity_;
  };
}
