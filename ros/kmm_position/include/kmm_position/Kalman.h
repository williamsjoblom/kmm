#pragma once

#include <Eigen/Dense>
#include "ros/ros.h"

typedef Eigen::Matrix<float, 6, 1> Vector6f;

namespace kmm_position {

  class Kalman {

  public:

    Kalman(float x, float y, float angle);

    // Predict next state based on steering signal.
    void predict(Eigen::Vector3f u);

    // Update the estimated state based on measured values.
    void lidar_measurement(Eigen::Vector3f y);
    void gyro_measurement(float y);

    // Current state.
    Eigen::Vector3f get_state();
    Eigen::Matrix3f get_state_cov();

    // Configure noise.
    void reset_state();

    void set_predict_noise(
      float linear_standard_deviation_m,
      float angular_standard_deviation_deg
    );

    void set_lidar_noise(
      float linear_standard_deviation_m,
      float angular_standard_deviation_deg
    );

    void set_gyro_noise(float angular);

  private:
    void set_initial_state_cov(
      float linear_standard_deviation_m,
      float angular_standard_deviation_deg
    );

    Eigen::Vector3f u_prev_;
    ros::Time predict_ts_;

    Vector6f state_, initial_state_;
    Vector6f state_cov_, initial_state_cov_;
    Eigen::Vector3f predict_noise_;
    Eigen::Vector3f lidar_noise_;
    float gyro_noise_;
  };

}
