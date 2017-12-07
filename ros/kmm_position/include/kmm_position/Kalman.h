#pragma once

#include <Eigen/Dense>
#include "ros/ros.h"

namespace kmm_position {

  class Kalman {

  public:

    Kalman(float x, float y, float angle);

    // Predict next state based on steering signal.
    void predict(const Eigen::Vector3f& u);

    // Update the estimated state based on measured values.
    void lidar_measurement(const Eigen::Vector3f y);
    void gyro_accel_measurement(const Eigen::Vector3f gyro_vec, const Eigen::Vector3f accel_vec);
    void gyro_measurement(const Eigen::Vector3f y);
    void accel_measurement(const Eigen::Vector3f y);

    // Current state.
    Eigen::Vector3f get_state();
    Eigen::Matrix3f get_state_cov();

    // Configure noise.
    void reset_state();
    void set_predict_noise(float linear, float angular);
    void set_lidar_noise(float linear, float angular);
    void set_gyro_noise(const Eigen::Matrix3f gyro_m);
    void set_accel_noise(const Eigen::Matrix3f accel_m);

  private:
    void set_state_cov(float linear, float angular);
    void set_cov(Eigen::Matrix3f& cov, float linear, float angular);
    void set_gyro_cov(Eigen::Matrix3f& cov, const Eigen::Matrix3f gyro_m);
    void set_accel_cov(Eigen::Matrix3f& cov, const Eigen::Matrix3f accel_m);

    Eigen::Vector3f state_;
    Eigen::Matrix3f state_cov_, predict_noise_, lidar_noise_, gyro_noise_, accel_noise_, I_;
    ros::Time predict_ts_, lidar_measurement_ts_, accel_gyro_measurement_ts_;
  };

}
