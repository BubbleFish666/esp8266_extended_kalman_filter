#include "ArduinoEigen/ArduinoEigenDense.h"

class ExtendedKalmanFilter {
private:
  // process noise covariance Q
  Eigen::Matrix3f Q_;

  // measurement noise covariance R
  Eigen::Matrix3f R_;

  // prediction
  // state estimate of last step
  Eigen::Vector3f x_c_k_1_;
  // estimate error covariance of last step
  Eigen::Matrix3f P_k_1_;
  // state Jacobian
  Eigen::Matrix3f A_k_;

  // correction
  // predicted state
  Eigen::Vector3f x_p_k_;
  // predicted measurement
  Eigen::Vector3f z_p_k_;
  // predicted estimate error covariance
  Eigen::Matrix3f P_p_k_;
  // measurement Jacobian
  Eigen::Matrix3f H_k_;
  // Kalman gain
  Eigen::Matrix3f K_k_;
  // measurement
  Eigen::Vector3f z_k_;

  // estimate (output of correction)
  // state estimate
  Eigen::Vector3f x_c_k_;
  // estimate error covariance
  Eigen::Matrix3f P_k_;

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

