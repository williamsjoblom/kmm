#include <iostream>
#include <stdexcept>

#include "Kalman.h"

KalmanFilter::KalmanFilter() {
    processNoiseCov_ << .05, .05, .0, .05, .05, .0, .0, .0, .0;
    measurementNoiseCov_ << 5;
    estErrorCov_ << .1, .1, .1, .1, 10000, 10, .1, 10, 100;
    I_.setIdentity();
}

void KalmanFilter::init(const Eigen::Vector3f& x0) {
    x_hat = x0;
    estErrorCov = estErrorCovInit;
}

void KalmanFilter::init() {
    x_hat.setZero();
    estErrorCov = estErrorCovInit;
}

void KalmanFilter::predict(const Eigen::Vector3f& u) {
    ros::Duration dt = ros::Time::now() - predict_ts_;
    x_hat_new = x_hat_ + dt * u;
    estErrorCov = estErrorCov_ + processNoiseCov_;
}

void KalmanFilter::updateWithLaser(const Eigen::Matrix3f y) {
    K_ = estErrorCov_*(estErrorCov_ + measurementNoiseCov_).inverse();
    x_hat_new_ += K * (y - output_*x_hat_new_);
    estErrorCov_ = (I_ - K*output_)*estErrorCov_;
    x_hat_ = x_hat_new_;
}