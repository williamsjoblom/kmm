#include <iostream>
#include <stdexcept>
#include "kmm_position/Kalman.h"
#include <ros/ros.h>
#include <math.h>

namespace kmm_position {

  const double pi = 3.1415926535897;

  Kalman::Kalman(float x, float y, float angle)
  {
    initial_state_ << x, y, angle, 0, 0, 0;
    set_initial_state_cov(0.1, 20);
    reset_state();
  }

  void Kalman::reset_state() {
    predict_ts_ = ros::Time::now();
    state_ = initial_state_;
    state_cov_ = initial_state_cov_;
    u_prev_ << 0, 0, 0;
  }

  void Kalman::set_initial_state_cov(
    float linear_standard_deviation_m,
    float angular_standard_deviation_deg
  ) {
    float linear_variance = std::pow(linear_standard_deviation_m, 2);
    float angular_variance = std::pow(angular_standard_deviation_deg * pi / 180, 2);
    initial_state_cov_ <<
      linear_variance,
      linear_variance,
      angular_variance,
      0, 0, 0;
  }

  void Kalman::set_predict_noise(
    float linear_standard_deviation_m,
    float angular_standard_deviation_deg
  ) {
    float linear_variance = std::pow(linear_standard_deviation_m, 2);
    float angular_variance = std::pow(angular_standard_deviation_deg * pi / 180, 2);
    predict_noise_ <<
      linear_variance,
      linear_variance,
      angular_variance;
  }

  void Kalman::set_lidar_noise(
    float linear_standard_deviation_m,
    float angular_standard_deviation_deg
  ) {
    float linear_variance = std::pow(linear_standard_deviation_m, 2);
    float angular_variance = std::pow(angular_standard_deviation_deg * pi / 180, 2);
    lidar_noise_ <<
      linear_variance,
      linear_variance,
      angular_variance;
  }

  void Kalman::set_gyro_noise(float angular_standard_deviation_deg) {
    float angular_variance = std::pow(angular_standard_deviation_deg * pi / 180, 2);
    gyro_noise_ = angular_variance;
  }

  void Kalman::predict(Eigen::Vector3f u) {
    float dt = (ros::Time::now() - predict_ts_).toSec();
    int hz = std::floor(1 / dt);
    predict_ts_ = ros::Time::now();
    if (hz > 1) {
      Eigen::Transform<float, 3, Eigen::Affine> t(Eigen::AngleAxis<float>(state_[2], Eigen::Vector3f(0, 0, 1)));
      Eigen::Vector3f global_u = t * u; // Rotate into global frame.

      Eigen::Vector3f a = (global_u - u_prev_) / dt;
      u_prev_ = global_u;

      state_[0] += state_[3] * dt + 0.5 * dt*dt * a[0];
      state_[1] += state_[4] * dt + 0.5 * dt*dt * a[1];
      state_[2] += state_[5] * dt + 0.5 * dt*dt * a[2];

      state_[3] += dt * a[0];
      state_[4] += dt * a[1];
      state_[5] += dt * a[2];

      state_cov_[0] += state_cov_[3] * dt + 0.5 * dt*dt * predict_noise_[0];
      state_cov_[1] += state_cov_[4] * dt + 0.5 * dt*dt * predict_noise_[1];
      state_cov_[2] += state_cov_[5] * dt + 0.5 * dt*dt * predict_noise_[2];

      state_cov_[3] += dt * predict_noise_[0];
      state_cov_[4] += dt * predict_noise_[1];
      state_cov_[5] += dt * predict_noise_[2];

      /*cov_increment(0, 0) *= abs(u[0]);
      cov_increment(1, 1) *= abs(u[1]);
      cov_increment(2, 2) *= abs(u[2]);*/

    } else {
      ROS_WARN("Too low frequency control signal to make position prediction: %d Hz", hz);
    }
  }

  void Kalman::lidar_measurement(Eigen::Vector3f y) {
    float K_x = state_cov_[0] / (state_cov_[0] + lidar_noise_[0]);
    float K_y = state_cov_[1] / (state_cov_[1] + lidar_noise_[1]);
    float K_0 = state_cov_[2] / (state_cov_[2] + lidar_noise_[2]);

    state_[0] += K_x * y[0];
    state_[1] += K_y * y[1];
    state_[2] += K_0 * y[2];

    state_cov_[0] *= (1 - K_x);
    state_cov_[1] *= (1 - K_y);
    state_cov_[2] *= (1 - K_0);
  }

  void Kalman::gyro_measurement(float y) {
    y -= state_[5];
    ROS_INFO_STREAM("state_cov_ --> " << state_cov_);
    float K = state_cov_[5] / (state_cov_[5] + gyro_noise_);
    state_[5] += K * y;
    state_cov_[5] *= (1 - K);
  }

  Eigen::Vector3f Kalman::get_state() {
    Eigen::Vector3f first_order_state(
      state_[0],
      state_[1],
      state_[2]
    );
    return first_order_state;
  }

  Eigen::Matrix3f Kalman::get_state_cov() {
    Eigen::Matrix3f first_order_cov;
    first_order_cov <<
      state_cov_[0], 0, 0,
      0, state_cov_[1], 0,
      0, 0, state_cov_[2];
    return first_order_cov;
  }

}
