#pragma once

#include <Eigen/Dense>
#include "ros/ros.h"

class KalmanFilter {

public:

  /**
  * Kalman filter constructor.
  */
  KalmanFilter();

  /**
  * Initialize the filter with initial states as zero.
  */
  void init();

  /**
  * Initialize the filter with a guess for initial states.
  */
  void init(const Eigen::Vector3f& x0);

  /**
  * Update the estimated state based on measured values. The
  * time step is assumed to remain constant.
  */
  void predict(const Eigen::Vector3f& u);

  /**
  * Update the estimated state based on measured values,
  * using the given time step and dynamics matrix.
  */
  void updateWithLaser(const Eigen::Vector3f y);

  /**
  * Return the current state.
  */
  Eigen::Vector3f getState() { return x_hat_; };

private:
  Eigen::MatrixXd processNoiseCov_, measurementNoiseCov_, estErrorCov_, K_, estErrorCovInit_;
  Eigen::MatrixXd I_;
  Eigen::Vector3f x_hat_, x_hat_new_;
  ros::Time predict_ts_;
};