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
  if (estimations.size() != ground_truth.size()
     || estimations.size() == 0) 
  {
    return rmse;
  }

  // TODO: accumulate squared residuals
  for (size_t i=0; i < estimations.size(); ++i) 
  {
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
  MatrixXd Hj(3,4);

  if(x_state.size() != 4)
  {
    return Hj;
  }

  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 

  // check division by zero
  
  // compute the Jacobian matrix
  float c1 = px * px + py * py;
  float c2 = sqrt(c1);
  float c3 = c1 * c2;

  if(fabs(c1) < 1e-4)
  {
    return Hj;
  }

  Hj << px / c2, py / c2, 0, 0,
        -py / c1, px / c1, 0, 0,
        py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

  return Hj;
}
