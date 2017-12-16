#include "kmm_navigation/PathFollower.hpp"

namespace kmm_navigation {

  const double pi = 3.1415926535897;

  PathFollower::PathFollower()
  : lowpass_vel_(0, 0)
  {
    error_p_constant_ = 0;
    max_velocity_ = 0;
  }

  void PathFollower::set_max_velocity(float max_velocity) {
    max_velocity_ = max_velocity;
  }

  void PathFollower::set_safe_velocity(float safe_velocity) {
    safe_velocity_ = safe_velocity;
  }

  void PathFollower::set_error_p_constant(float error_p_constant) {
    error_p_constant_ = error_p_constant;
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
    if (path.size() < 2) {
      vel[0] = 0;
      vel[1] = 0;
      has_reached_target = true;
      return;
    }

    double offset_distance = std::numeric_limits<double>::infinity();
    Eigen::Vector2f offset_vector(0,0);
    Eigen::Vector2f forward_vector(0, 0);
    int closest_index = 0;

    for (int i = 0 ; i < path.size(); i++){
      Eigen::Vector2f robot_vector = robot_position - path[i];
      Eigen::Vector2f path_vector = path[i+1] - path[i];

      if (i != path.size() - 1) {
        double proj_factor = robot_vector.dot(path_vector)/path_vector.squaredNorm();

        if (proj_factor < 1 && (proj_factor >= 0 || i == 0)) {
          //An vector from the robot back to the path
          Eigen::Vector2f  curr_offset_vector = proj_factor * path_vector - robot_vector;

          if (curr_offset_vector.norm() < offset_distance) {
            // This is the current closest point on the path to the robot
            offset_distance = curr_offset_vector.norm();
            offset_vector = curr_offset_vector;
            forward_vector = path_vector.normalized();
            closest_index = i;
          }
        }
      }

      if (i != 0) {
        if (robot_vector.norm() < offset_distance) {
            offset_distance = robot_vector.norm();
            offset_vector = robot_vector * -1;
            forward_vector = path_vector.normalized();
            closest_index = i;
        }
      }
    }

    // Checks if the robot has reached the target
    has_reached_target = false;
    Eigen::Vector2f destination = path[path.size() - 1];
    float distance_to_destination = (destination - robot_position).norm();
    if (distance_to_destination < 0.05) {
      has_reached_target = true;
    }

    // Check if close to target.
    bool is_close_to_target = distance_to_destination < 0.15;

    // Check if path is curvning.
    bool is_path_curvning = false;
    int look_ahead_index = closest_index + 5;
    if (look_ahead_index < path.size()) {
      Eigen::Vector2f v1 = path[closest_index + 1] - path[closest_index];
      Eigen::Vector2f v2 = path[look_ahead_index] - path[closest_index];
      float angle_deg = std::acos(v1.dot(v2) / (v1.norm() * v2.norm())) * (180 / pi);
      ROS_INFO("path angle: %.2f", angle_deg);
      is_path_curvning = angle_deg > 3;
    }


    // Adjust speed and slow down in difficult situations.
    float forward_velocity = max_velocity_;
    if (is_close_to_target || is_path_curvning) {
      forward_velocity = safe_velocity_;
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
      Eigen::Vector2f forward_vel_component = forward_vector * forward_velocity;

      // This is the component that minimizes the error.
      if (offset_vector.norm() > 0.1) {
        offset_vector = offset_vector.normalized() * 0.1;
      }
      Eigen::Vector2f offset_vel_component = offset_vector * error_p_constant_;

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
