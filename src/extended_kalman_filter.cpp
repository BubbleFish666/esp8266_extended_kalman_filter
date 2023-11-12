#include "extended_kalman_filter.h"
#include <math.h>

/*constructor
  @param x_0 initial state
*/
ExtendedKalmanFilter::ExtendedKalmanFilter(Eigen::Vector3f x_0)
    : x_c_k_1_(x_0) {
  // process noise covariance
  Q_ << 0.5, 0, 0.01,
        0, 0.5, 0.01,
        0.01, 0.01, 0.3;
  // measurement noise covariance
  R_ << 0.5, 0, 0.1,
        0, 0.5, 0.1,
        0.1, 0.1, 0.6;
  // initial estimate error covariance
  P_k_1_ = Eigen::Matrix3f::Identity();
}

/*state function of the system*/
void ExtendedKalmanFilter::stateFcn() {
  x_p_k_(0) = x_c_k_1_(0);
  x_p_k_(1) = x_c_k_1_(1);
  x_p_k_(2) = std::atan2(x_c_k_1_(1), x_c_k_1_(0));
}

/*Jacobian of the state function of the system*/
void ExtendedKalmanFilter::stateJacFcn() {
  A_k_ << 1, 0, 0,
          0, 1, 0,
          - x_c_k_1_(1) / (x_c_k_1_(0) * x_c_k_1_(0) + x_c_k_1_(1) * x_c_k_1_(1)),
          x_c_k_1_(0) / (x_c_k_1_(0) * x_c_k_1_(0) + x_c_k_1_(1) * x_c_k_1_(1)),
          0;
}

/*measurement function of the system*/
void ExtendedKalmanFilter::measureFcn() {
  z_p_k_ = x_p_k_;
}

/*Jacobian of the measurement function of the system*/
void ExtendedKalmanFilter::measureJacFcn() {
  H_k_ = Eigen::Matrix3f::Identity();
}

/*do prediction*/
void ExtendedKalmanFilter::predict() {
  // calculate _x_p_k
  stateFcn();
  // calculate _A_k
  stateJacFcn();
  // calculate _P_p_k
  P_p_k_ = A_k_ * P_k_1_ * A_k_.transpose() + Q_;
}

/*do correction*/
void ExtendedKalmanFilter::correct() {
  // calculate _H_k
  measureJacFcn();
  // calculate _K_k
  K_k_ = P_p_k_ * H_k_.transpose() *
         (H_k_ * P_p_k_ * H_k_.transpose() + R_).inverse();
  // calculate _z_p_k
  measureFcn();
  // calculate _x_c_k
  x_c_k_ = x_p_k_ + K_k_ * (z_k_ - z_p_k_);
  // calculate _P_k
  P_k_ = (Eigen::Matrix3f::Identity() - K_k_ * H_k_) * P_p_k_;
}

/*do estimation for the current step
  @param z_k measurement
  @returns estimate of state
*/
Eigen::Vector3f ExtendedKalmanFilter::estimate(Eigen::Vector3f z_k) {
  z_k_ = z_k;
  predict();
  correct();
  x_c_k_1_ = x_c_k_;
  P_k_1_ = P_k_;
  return x_c_k_;
}