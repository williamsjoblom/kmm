#pragma once
#include <Eigen/Dense>
#include "ros/ros.h"


namespace kmm_navigation {

  class PathFollower {
  public:
    PathFollower();
    void get_velocity(
      const std::vector<Eigen::Vector2f>& path,
      const Eigen::Vector2f& robot_position,
      Eigen::Vector2f& vel,
      bool& has_reached_target
    );
    void set_error_p_constant(float error_p_constant);
    void set_max_velocity(float max_velocity);
    void set_filter_constant(float filter_constant);

  private:
    float error_p_constant_;
    float max_velocity_;
    float filter_constant_;
    Eigen::Vector2f lowpass_vel_;
    ros::Time vel_ts_;
  };
}
