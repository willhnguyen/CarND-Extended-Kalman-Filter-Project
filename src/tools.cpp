#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

  VectorXd rmse = VectorXd(4);
  rmse << 0, 0, 0, 0;
  int num_data = estimations.size();

  // Check if we can proceed without issues
  if (num_data == 0 || num_data != ground_truth.size()) {
    std::cout << "ERROR: Either the estimations vector is empty or not the same size as the ground_truth." << std::endl;
    return rmse;
  }

  // Sum up the square of the errors
  for (int i = 0; i < num_data; ++i) {
    VectorXd diff = estimations[i] - ground_truth[i];

    diff = diff.array().square();
    rmse += diff;
  }

  // Calculate the mean error
  rmse /= (float) num_data;

  // Calculate square root of mean error
  rmse.array().sqrt();

  // // Print rmse
  // std::cout << "RMSE: " << rmse.transpose() << std::endl;

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

  float px = x_state(0),
        py = x_state(1),
        vx = x_state(2),
        vy = x_state(3);
  MatrixXd Hj = MatrixXd(3, 4);
  Hj << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;

  // Calculate the Jacobian matrix

  float pmag = px * px + py * py;
  float pdist = sqrt(pmag);
  float pmag32 = pmag * pdist;

  if (abs(pmag32) <= 0.0001)
  {
    std::cout << "ERROR: Cannot calculate Jacobian because px and py are 0." << std::endl;
  }

  Hj(0, 0) = px / pdist;
  Hj(0, 1) = py / pdist;
  Hj(1, 0) = -py / pmag;
  Hj(1, 1) = px / pmag;
  Hj(2, 0) = py * (vx * py - vy * px) / (pmag32);
  Hj(2, 1) = px * (vy * px - vx * py) / (pmag32);
  Hj(2, 2) = px / pdist;
  Hj(2, 3) = py / pdist;

  return Hj;
}
