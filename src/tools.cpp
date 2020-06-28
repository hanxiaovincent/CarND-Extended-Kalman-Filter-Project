#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // TODO: YOUR CODE HERE
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size

  // TODO: accumulate squared residuals
  for (int i=0; i < estimations.size(); ++i) {
    // ... your code here
    VectorXd diff = estimations[i] - ground_truth[i];
    VectorXd diff2 = diff.array() * diff.array();
    rmse += diff2;
  }

  // TODO: calculate the mean
  rmse /= estimations.size();
  // TODO: calculate the squared root
  rmse = rmse.array().sqrt();
  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj = MatrixXd::Zero(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 

  // check division by zero
  
  // compute the Jacobian matrix
  float px2 = px * px;
  float py2 = py * py;
  float px2py2sum = px2 + py2;

  if(fabs(px2py2sum) < 1e-6)
  {
    return Hj;
  }

  Hj(0, 0) = px / sqrt(px2py2sum);
  Hj(0, 1) = py / sqrt(px2py2sum);
  Hj(1, 0) = - py / px2py2sum;
  Hj(1, 1) = px / px2py2sum;
  Hj(2, 0) = py * (vx * py - vy * px) / pow(px2py2sum, 1.5);
  Hj(2, 1) = px * (vy * px - vx * py) / pow(px2py2sum, 1.5);
  Hj(2, 2) = Hj(0, 0);
  Hj(2, 3) = Hj(0, 1);

  return Hj;
}
