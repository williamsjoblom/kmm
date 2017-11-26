#include <iostream>
#include <stdexcept>
#include "kmm_position/Kalman.h"
#include <ros/ros.h>

namespace kmm_position {

  const double pi = 3.1415926535897;

  Kalman::Kalman(float x, float y, float angle)
    : state_(x, y, angle)
  {
    I_.setIdentity();

    set_state_cov(0.2, 20 * pi / 180);
    set_predict_noise(0.02, 0.01);
    set_lidar_noise(1, 5 * pi / 180);

    predict_ts_ = ros::Time::now();
    lidar_measurement_ts_ = ros::Time::now();
    accel_gyro_measurement_ts_ = ros::Time::now();
  }

  void Kalman::set_cov(Eigen::Matrix3f& cov, float linear, float angular) {
    float l = linear * linear;
    float a = angular * angular;
    cov <<
      l, 0, 0,
      0, l, 0,
      0, 0, a;
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

  void Kalman::predict(const Eigen::Vector3f& u) {
      float dt = (ros::Time::now() - predict_ts_).toSec();
      predict_ts_ = ros::Time::now();
      if (dt < 1./20) {
        state_ += dt * u;
        state_cov_ += predict_noise_;
      } else {
        ROS_WARN_THROTTLE(1, "Too low frequency control signal to make position prediction");
      }
  }

  void Kalman::lidar_measurement(const Eigen::Vector3f y) {
      Eigen::Matrix3f K = state_cov_ * (state_cov_ + lidar_noise_).inverse();
      state_ += K * y;
      state_cov_ *= (I_ - K);
  }

  void Kalman::accel_gyro_measurement(const Eigen::Vector3f y) {

  }

  Eigen::Vector3f Kalman::get_state() {
    return state_;
  }

  Eigen::Matrix3f Kalman::get_state_cov() {
    return state_cov_;
  }

}
