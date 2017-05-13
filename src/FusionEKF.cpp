#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  // create a 4-D state vector
  ekf_.x_ = VectorXd(4);

  // state covariance matrix P
  ekf_.P_ = MatrixXd(4,4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

  // measurement matrix
  ekf_.H_ = MatrixXd(2,4);
  ekf_.H_ << 1, 0, 0, 0,
             0, 1, 0, 0; 

  // initial transition matrix F_
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;
  
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << 0, 0, 0, 0,
             0, 0, 0, 0,
             0, 0, 0, 0,
             0, 0, 0, 0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      x = rho * cos(theta)
      y = rho * sin(theta)
      rhodot_x  = rhodot * cos(theta)
      rhodot_y  = rhodot * sin(theta)
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0] * cos( measurement_pack.raw_measurements_[1] ),\
                 measurement_pack.raw_measurements_[0] * sin( measurement_pack.raw_measurements_[1] ),\
                 0, 0;
                
      ekf_.R_ = R_radar_;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0.0, 0.0;

      ekf_.R_ = R_laser_;
            
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  // set the appropiate measurement covariance matrix
  if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
     ekf_.R_    = R_laser_;
  } 
  else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
     ekf_.R_    = R_radar_;
  } 
  

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt * dt3;
  
  const float noise_ax = 9;
  const float noise_ay = 9;

  // modify the F matrix to integrate time
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
 
  ekf_.Q_ << (dt4/4 * noise_ax), 0, (dt3/2 * noise_ax), 0,
             0, (dt4/4 * noise_ay), 0, (dt3/2 * noise_ay),
             (dt3/2 * noise_ax), 0, (dt2 * noise_ax), 0,
             0, (dt3/2 * noise_ay), 0, (dt2 * noise_ay);


  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
