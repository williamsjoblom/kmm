#include <iostream>
#include <stdexcept>
#include "kmm_position/Kalman.h"
#include <ros/ros.h>
#include <math.h>

namespace kmm_position {

  const double pi = 3.1415926535897;

  Kalman::Kalman(float x, float y, float angle)
    : state_(x, y, angle)
  {
    I_.setIdentity();

    float initial_linear_error_m = 0.2;
    float initial_angular_error_deg = 20;
    set_state_cov(initial_linear_error_m, initial_angular_error_deg);

    predict_ts_ = ros::Time::now();
    lidar_measurement_ts_ = ros::Time::now();
    accel_gyro_measurement_ts_ = ros::Time::now();
  }

  void Kalman::set_cov(
    Eigen::Matrix3f& cov,
    float linear_standard_deviation_m,
    float angular_standard_deviation_deg
  ) {
    float linear_variance = std::pow(linear_standard_deviation_m, 2);
    float angular_variance = std::pow(angular_standard_deviation_deg * pi / 180, 2);
    cov <<
      linear_variance, 0              , 0,
      0              , linear_variance, 0,
      0              , 0              , angular_variance;
  }

  void Kalman::reset_state() {
    Eigen::Vector3f reset_state(0.2, 0.2, 0);
    state_ = reset_state;
    set_state_cov(0.2, 20 * pi / 180);
    set_predict_noise(0.05, 0.03);
    set_lidar_noise(0.1, 0.5 * pi / 180);
  }

  void Kalman::set_state_cov(float linear, float angular) {
    set_cov(state_cov_, linear, angular);
  }

  void Kalman::set_predict_noise(float linear, float angular) {
    set_cov(predict_noise_, linear, angular);
  }


  void Kalman::set_lidar_noise(float linear, float angular) {
    set_cov(lidar_noise_, linear, angular);
  }

  // New section maintained by mr Hamp
  // No new object members added yet.

  void Kalman::set_gyro_cov(
    Eigen::Matrix3f& cov,
    const Eigen::Matrix3f gyro_m
  ) {
    cov = gyro_m;
    }

  void Kalman::set_accel_cov(
    Eigen::Matrix3f& cov,
    const Eigen::Matrix3f accel_m
  ){}

  void Kalman::set_gyro_noise(const Eigen::Matrix3f gyro_m) {
    set_gyro_cov(gyro_noise_, gyro_m);
  }

  void Kalman::set_accel_noise(const Eigen::Matrix3f accel_m) {
    // Evaluate if acccelero should be used due to behaviour when moving.
    set_accel_cov(accel_noise_, accel_m);
  }

  void Kalman::predict(Eigen::Vector3f u) {
      float dt = (ros::Time::now() - predict_ts_).toSec();
      int hz = std::floor(1 / dt);
      predict_ts_ = ros::Time::now();
      if (hz > 1) {
        Eigen::Transform<float, 3, Eigen::Affine> t(Eigen::AngleAxis<float>(state_[2], Eigen::Vector3f(0, 0, 1)));
        Eigen::Vector3f global_u = t * u; // Rotate into global frame.

        state_ += dt * global_u;

        Eigen::Matrix3f cov_increment = predict_noise_ * dt;
        /*cov_increment(0, 0) *= abs(u[0]);
        cov_increment(1, 1) *= abs(u[1]);
        cov_increment(2, 2) *= abs(u[2]);*/
        state_cov_ += cov_increment;
      } else {
        ROS_WARN("Too low frequency control signal to make position prediction: %d Hz", hz);
      }
  }

  void Kalman::lidar_measurement(Eigen::Vector3f y) {
      Eigen::Matrix3f K = state_cov_ * (state_cov_ + lidar_noise_).inverse();
      state_ += K * y;
      state_cov_ *= (I_ - K);
  }

  void Kalman::gyro_accel_measurement(
    const Eigen::Vector3f gyro_vec,
    const Eigen::Vector3f accel_vec
  ) {

    gyro_measurement(gyro_vec);
    //accel_measurement(accel_vec);
  }

  void Kalman::gyro_measurement(Eigen::Vector3f y) {
    y[2] -= state_[2];
    ROS_INFO_STREAM("y: " << y);
    ROS_INFO_STREAM("state_cov_: " << state_cov_);
    ROS_INFO_STREAM("gyro_noise_: " << gyro_noise_);
    //Eigen::Matrix3f K = state_cov_ * (state_cov_ + gyro_noise_).inverse();
    Eigen::Matrix3f K;
    K <<
      0, 0, 0,
      0, 0, 0,
      0, 0, state_cov_(8) / (state_cov_(8) + gyro_noise_(8));
    state_ += K * y;
    state_cov_ *= (I_ - K);
    ROS_INFO_STREAM("K: " << K);
    ROS_INFO_STREAM("state_: " << state_);
    ROS_INFO_STREAM("state_cov_: " << state_cov_);
  }

  void Kalman::accel_measurement(Eigen::Vector3f y) {
    // Evaluate if needed due to the behaviour of data when running.
  }

  Eigen::Vector3f Kalman::get_state() {
    return state_;
  }

  Eigen::Matrix3f Kalman::get_state_cov() {
    return state_cov_;
  }

}
