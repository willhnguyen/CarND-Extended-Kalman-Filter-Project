#include "kalman_filter.h"
#include <iostream>
#define PI 3.141592

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */

  // x' = Fx + u; u is 0 in this case
  x_ = F_ * x_;

  // P' = F*P*F_T + Q
  P_ = F_ * P_ * F_.transpose() + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  // y = z - Hx
  VectorXd y = z - H_ * x_;

  UpdateUniversal(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  // y = z - h(x)
  VectorXd hx = VectorXd(3); // Convert x_ with h(x) from 4d cartesian to 3d polar
  float px = x_(0),
        py = x_(1),
        vx = x_(2),
        vy = x_(3);
  float rho = sqrt(px*px + py*py),
        phi = atan2(py, px),
        rho_dot = (px*vx + py*vy) / rho;

  // Set hx values
  hx << rho, phi, rho_dot;

  // Calculate y
  VectorXd y = z - hx;

  // Ensure phi is between pi and -pi
  while (y(1) > M_PI) {
    y(1) -= M_PI * 2;
  }
  while (y(1) < -M_PI) {
    y(1) += M_PI * 2;
  }

  UpdateUniversal(y);
}

void KalmanFilter::UpdateUniversal(const VectorXd &y) {
  // S = H*P*H_T + R
  MatrixXd H_T = H_.transpose();
  MatrixXd S = (H_ * P_ * H_T) + R_;

  // K = P*H_T*S_inv
  MatrixXd K = P_ * H_T * S.inverse();

  // x' = x + Ky
  x_ = x_ + K * y;

  // P' = (I-KH)*P
  MatrixXd I = MatrixXd::Identity(4,4);
  P_ = (I - K * H_) * P_;
}
