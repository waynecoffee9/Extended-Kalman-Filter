#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  // initialize rmse vector
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  if (estimations.size()==0)
  {
      cout << "Error - estimation vector size is zero" << endl;
      return rmse;
  }
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size())
  {
      cout << "Error - estimation and ground truth vector sizes don't match" << endl;
      return rmse;
  }
  // accumulate squared residuals
  for (int i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
  }
  // calculate the mean
  rmse = rmse/estimations.size();
  // calculate the squared root
  rmse = sqrt(rmse.array());
  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  // check division by zero
  if (px*px + py*py < 0.0001) {
    px = 0.001;
    py = 0.001;
  }
  float px2 = px*px;
  float py2 = py*py;
  float pxy2 = px2 + py2;

  // compute the Jacobian matrix
  Hj << px/pow(pxy2, 0.5), py/pow(pxy2, 0.5), 0, 0,
  		-py/pxy2, px/pxy2, 0, 0,
  		py*(vx*py - vy*px)/pow(pxy2, 1.5), px*(vy*px - vx*py)/pow(pxy2, 1.5), px/pow(pxy2, 0.5), py/pow(pxy2, 0.5);

  return Hj;
}
