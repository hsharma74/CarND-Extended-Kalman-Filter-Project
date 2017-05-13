#include "kalman_filter.h" 
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

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
    * predict the state
  */

  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */

  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
} 
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations
  */
  cout << "In KalmanFilter::UpdateEKF()" << endl;
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  // calculate the h(x') function
  float rho =  sqrt( pow(px, 2) + pow(py, 2) );
  // check for tininess of px for divide_by_zero error
  if ( fabs(px) < 0.0001 ) {
     px = 0.0001;
  }

  float phi =  atan2( py , px );
  // check for tininess of rho for divide_by_zero error
  if ( fabs(rho) < 0.0001 ) {
     rho = 0.0001;
  }

  float rho_dot =  ( px * vx + py * vy ) / rho;  

  Eigen::VectorXd h_x_ = VectorXd(3);
  h_x_ << rho, phi, rho_dot;
 
  VectorXd y   = z - h_x_; 
  // normalize angle in y to be between -PI and +PI
  y(1) = y(1) - 2 * M_PI * std::floor( (y(1) + M_PI) / (2*M_PI) ); 

  Tools tools;
  MatrixXd Hj  = tools.CalculateJacobian( x_ ); 
  MatrixXd Hjt = Hj.transpose();
  MatrixXd S   = Hj * P_ * Hjt + R_;
  MatrixXd Si  = S.inverse();
  MatrixXd PHjt = P_ * Hjt;
  MatrixXd K   = PHjt * Si;

  // new estimate 
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj) * P_;

}
