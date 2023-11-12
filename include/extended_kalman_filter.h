#include "ArduinoEigen/ArduinoEigenDense.h"

class ExtendedKalmanFilter {
private:
  // process noise covariance Q
  Eigen::Matrix3f _Q;

  // measurement noise covariance R
  Eigen::Matrix3f _R;

  // prediction
  // state estimate of last step
  Eigen::Vector3f _x_c_k_1;
  // estimate error covariance of last step
  Eigen::Matrix3f _P_k_1;
  // state Jacobian
  Eigen::Matrix3f _A_k;

  // correction
  // predicted state
  Eigen::Vector3f _x_p_k;
  // predicted measurement
  Eigen::Vector3f _z_p_k;
  // predicted estimate error covariance
  Eigen::Matrix3f _P_p_k;
  // measurement Jacobian
  Eigen::Matrix3f _H_k;
  // Kalman gain
  Eigen::Matrix3f _K_k;
  // measurement
  Eigen::Vector3f _z_k;

  // estimate (output of correction)
  // state estimate
  Eigen::Vector3f _x_c_k;
  // estimate error covariance
  Eigen::Matrix3f _P_k;

  void stateFcn();
  void stateJacFcn();
  void measureFcn();
  void measureJacFcn();

public:
  ExtendedKalmanFilter(Eigen::Vector3f x_0);
  void predict();
  void correct();
  Eigen::Vector3f estimate(Eigen::Vector3f z_k);
};

