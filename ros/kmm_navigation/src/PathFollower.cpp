#include "kmm_navigation/PathFollower.hpp"

namespace kmm_navigation {

  PathFollower::PathFollower()
  : lowpass_vel_(0, 0)
  {
    error_p_constant_ = 0;
    max_velocity_ = 0;
  }

  void PathFollower::set_error_p_constant(float error_p_constant) {
    error_p_constant_ = error_p_constant;
  }

  void PathFollower::set_max_velocity(float max_velocity) {
    max_velocity_ = max_velocity;
  }

  void PathFollower::set_filter_constant(float filter_constant) {
    filter_constant_ = filter_constant;
  }

  /*
   * Calculates and controls the next velocity of the robot so it follows the
   * desired path (path). The new velocity is saved in vel.
   */
  void PathFollower::get_velocity(
    const std::vector<Eigen::Vector2f>& path,
    const Eigen::Vector2f& robot_position,
    Eigen::Vector2f& vel,
    bool& has_reached_target
  )
  {
    double offset_distance = std::numeric_limits<double>::infinity();
    Eigen::Vector2f offset_vector(0,0);
    Eigen::Vector2f curr_offset_vector;
    Eigen::Vector2f path_vector;
    Eigen::Vector2f robot_vector;
    Eigen::Vector2f forward_vector(0, 0);
    double proj_factor;
    for (int i = 0 ; i < path.size(); i++){
      robot_vector = robot_position - path[i];
      if (i != path.size() - 1) {
        path_vector = path[i+1] - path[i];
        proj_factor = robot_vector.dot(path_vector)/path_vector.squaredNorm();
        if (proj_factor < 1 && proj_factor >= 0) {
          // Have found a point on path that is close to the robot position

          //An vector from the robot back to the path
          curr_offset_vector = proj_factor*path_vector - robot_vector;
          if (curr_offset_vector.norm() < offset_distance){
            // This is the current closest point on the path to the robot
            offset_distance = curr_offset_vector.norm();
            offset_vector = curr_offset_vector;
            forward_vector = path_vector.normalized();
          }
        }
      }
      if (robot_vector.norm() < offset_distance) {
          offset_distance = robot_vector.norm();
          offset_vector = robot_vector * -1;
          forward_vector = path_vector.normalized();
      }
    }

    // Checks if the robot has reached the target
    has_reached_target = false;
    if (path.size() > 0) {
      Eigen::Vector2f destination = path[path.size() - 1];
      float distance_to_destination = (destination - robot_position).norm();
      if (distance_to_destination < 0.05) {
        has_reached_target = true;
      }
    }

    if (has_reached_target) {
      // The target has been reached and the robot shouldn't move
      lowpass_vel_[0] = 0;
      lowpass_vel_[1] = 0;
      vel[0] = 0;
      vel[1] = 0;
    } else {
      // The robot has not reached the target and the velocity is to be set

      // This is compontent that takes the robot forward along the path.
      Eigen::Vector2f forward_vel_component = forward_vector * max_velocity_;

      // This is the component that minimizes the error.
      Eigen::Vector2f offset_vel_component = offset_vector * error_p_constant_;
      if (offset_vel_component.norm() > max_velocity_) {
        offset_vel_component = offset_vel_component.normalized() * max_velocity_;
      }

      // Lowpass filter forward velocity.
      Eigen::Vector2f total_vel(0, 0);
      float dt = (ros::Time::now() - vel_ts_).toSec();
      int hz = std::floor(1 / dt);
      vel_ts_ = ros::Time::now();
      if (hz > 1) {
        lowpass_vel_ += (forward_vel_component - lowpass_vel_) * (dt / (dt + filter_constant_));
        total_vel = offset_vel_component + lowpass_vel_;
      } else {
        ROS_WARN("Too low frequency control signal to lowpass velocity: %d Hz", hz);
      }
      vel = total_vel;
    }
  }
}
