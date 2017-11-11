#include <iostream>
#include <stdexcept>

#include "kmm_position/Kalman.h"

KalmanFilter::KalmanFilter() {
    dt_ << 0, 0, 0;
    output_ << 0, 0, 0;
    processNoiseCov_ << .05, .05, .0, .05, .05, .0, .0, .0, .0;
    measurementNoiseCov_ << 5;
    estErrorCov_ << .1, .1, .1, .1, 10000, 10, .1, 10, 100;
    I_.setIdentity();
}

void KalmanFilter::init(const Eigen::Vector3f& x0) {
    x_hat_ = x0;
}

void KalmanFilter::init() {
    x_hat_.setZero();
}

void KalmanFilter::predict(const Eigen::Vector3f& u) {
    ros::Duration dt = ros::Time::now() - predict_ts_;
    dt_ << 1 , 1, 1;
    x_hat_new_ = x_hat_ + dt_ * u;
    estErrorCov_ = estErrorCov_ + processNoiseCov_;
}

void KalmanFilter::updateWithLaser(const Eigen::Matrix3f y) {
    K_ = estErrorCov_*(estErrorCov_ + measurementNoiseCov_).inverse();
    x_hat_new_ = K_ * (y - output_*x_hat_new_);
    estErrorCov_ = (I_ - K_*output_)*estErrorCov_;
    x_hat_ = x_hat_new_;
}
