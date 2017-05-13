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
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  // check that estimation vector size in on-zero
  // check that estimation and ground_truth vectors are equal in size
  assert( estimations.size() != 0 );
  assert( estimations.size() == ground_truth.size() );

  // accumulate squared residuals
  for ( int i = 0; i < estimations.size(); i++ ) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // calculate the mean of the squared residuals
  VectorXd mean(4);
  mean = rmse / estimations.size();

  // now get the square root of the mean of squared residuals
  rmse = mean.array().sqrt(); 
  
  return rmse;
}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3, 4);
  // recover state parameters 
  float px = x_state[0];
  float py = x_state[1];
  float vx = x_state[2];
  float vy = x_state[3];

  float px_py_square   = pow(px, 2) + pow(py, 2); 
  float px_py_sqrt = sqrt(px_py_square);
  float px_py_sqrt_pow3 = px_py_square * px_py_sqrt;
 
  // check division by zero (and very small numbers); we only need to check 
  // px^2 + py^2 since that is the term that seeds all denominators
  if ( fabs(px_py_square) < 0.0001 ) {
    perror( "Jacobian - illegal division by zero" );
    return Hj;
  }

  // calculate the various terms of the Jacobian matrix
  // a smart compiler should be able to find redundant values in code and reuse
  float term_2_0 = py * ( vx*py - vy*px ) / px_py_sqrt_pow3;
  float term_2_1 = px * ( -vx*py + vy*px ) / px_py_sqrt_pow3;
  float term_0_0 = px / px_py_sqrt;
  float term_0_1 = py / px_py_sqrt;
  float term_1_0 = -py / px_py_square;
  float term_1_1 = px / px_py_square;

  Hj << term_0_0, term_0_1, 0, 0,
        term_1_0, term_1_1, 0, 0,
        term_2_0, term_2_1, term_0_0, term_0_1; 

  return Hj;

}
