#include "extended_kalman_filter.h"
#include <math.h>

/*constructor
  @param x_0 initial state
*/
ExtendedKalmanFilter::ExtendedKalmanFilter(Eigen::Vector3f x_0)
    : _x_c_k_1(x_0) {
  // process noise covariance
  _Q << 0.5, 0, 0.01,
        0, 0.5, 0.01,
        0.01, 0.01, 0.3;
  // measurement noise covariance
  _R << 0.5, 0, 0.1,
        0, 0.5, 0.1,
        0.1, 0.1, 0.6;
  // initial estimate error covariance
  _P_k_1 = Eigen::Matrix3f::Identity();
}

/*state function of the system*/
void ExtendedKalmanFilter::stateFcn() {
  _x_p_k(0) = _x_c_k_1(0);
  _x_p_k(1) = _x_c_k_1(1);
  _x_p_k(2) = std::atan2(_x_c_k_1(1), _x_c_k_1(0));
}

/*Jacobian of the state function of the system*/
void ExtendedKalmanFilter::stateJacFcn() {
  _A_k << 1, 0, 0,
          0, 1, 0,
          -_x_c_k_1(1)/(_x_c_k_1(0)*_x_c_k_1(0) + _x_c_k_1(1)*_x_c_k_1(1)),
          _x_c_k_1(0)/(_x_c_k_1(0)*_x_c_k_1(0) + _x_c_k_1(1)*_x_c_k_1(1)),
          0;
}

/*measurement function of the system*/
void ExtendedKalmanFilter::measureFcn() {
  _z_p_k = _x_p_k;
}

/*Jacobian of the measurement function of the system*/
void ExtendedKalmanFilter::measureJacFcn() {
  _H_k = Eigen::Matrix3f::Identity();
}

/*do prediction*/
void ExtendedKalmanFilter::predict() {
  // calculate _x_p_k
  stateFcn();
  // calculate _A_k
  stateJacFcn();
  // calculate _P_p_k
  _P_p_k = _A_k * _P_k_1 * _A_k.transpose() + _Q;
}

/*do correction*/
void ExtendedKalmanFilter::correct() {
  // calculate _H_k
  measureJacFcn();
  // calculate _K_k
  _K_k = _P_p_k * _H_k.transpose() *
         (_H_k * _P_p_k * _H_k.transpose() + _R).inverse();
  // calculate _z_p_k
  measureFcn();
  // calculate _x_c_k
  _x_c_k = _x_p_k + _K_k * (_z_k - _z_p_k);
  // calculate _P_k
  _P_k = (Eigen::Matrix3f::Identity() - _K_k * _H_k) * _P_p_k;
}

/*do estimation for the current step
  @param z_k measurement
  @returns estimate of state
*/
Eigen::Vector3f ExtendedKalmanFilter::estimate(Eigen::Vector3f z_k) {
  _z_k = z_k;
  predict();
  correct();
  _x_c_k_1 = _x_c_k;
  _P_k_1 = _P_k;
  return _x_c_k;
}