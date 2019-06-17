#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
  ekf_.x_ = VectorXd(4);   // 4D state vector.
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.F_ = MatrixXd(4, 4);
  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */

  // state covariance matrix
  ekf_.P_ << 0.1, 0, 0, 0,
  		0, 0.1, 0, 0,
  		0, 0, 1000, 0,
  		0, 0, 0, 1000;
  
  // measurement matrix
  H_laser_ << 1, 0, 0, 0,
  			  0, 1, 0, 0;
  
  // state transition matrix
  ekf_.F_ << 1, 0, 1, 0,
  		0, 1, 0, 1,
  		0, 0, 1, 0,
  		0, 0, 0, 1;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
	x_his = MatrixXd(4,10);
	x_his.fill(0.0);
	x_lowpass = MatrixXd(2,10);
	x_lowpass.fill(0.0);
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
	  float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rhodot = measurement_pack.raw_measurements_[2];
      // include velocity for radar measurement
      ekf_.x_ << rho * cos(phi), rho * sin(phi), rhodot * cos(phi), rhodot * sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0.0, 0.0;
    }
	previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  float dt_2 = dt*dt;
  float dt_3 = dt_2*dt;
  float dt_4 = dt_3*dt;
  
  // Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  
  // set the acceleration noise components
  float noise_ax = 9;
  float noise_ay = 9;
  
  // set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    ekf_.R_ = R_radar_;
    Tools tools;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // TODO: Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }
  // Low pass filter
  x_his.col(0) = x_his.col(1);
  x_his.col(1) = x_his.col(2);
  x_his.col(2) = x_his.col(3);
  x_his.col(3) = x_his.col(4);
  x_his.col(4) = x_his.col(5);
  x_his.col(5) = x_his.col(6);
  x_his.col(6) = x_his.col(7);
  x_his.col(7) = x_his.col(8);
  x_his.col(8) = x_his.col(9);
  x_his.col(9) = ekf_.x_;
  double CUTOFF = 4.0;				// cut off frequency
  double RC = 1.0/(CUTOFF*2*3.14);
  double alpha = dt/(RC+dt);
  x_lowpass.row(0) = x_his.row(2);
  x_lowpass.row(1) = x_his.row(3);
  // low pass filter implementation for both x, and y velocities
  for(int i = 1; i < 10; ++i)
  { 
    x_lowpass(0,i) = x_lowpass(0,i-1) + (alpha*(x_his(2,i) - x_lowpass(0,i-1)));
  }
  for(int i = 1; i < 10; ++i)
  { 
    x_lowpass(1,i) = x_lowpass(1,i-1) + (alpha*(x_his(3,i) - x_lowpass(1,i-1)));
  }
  double a1 = (x_his(2,8) - x_his(2,7))/dt;
  double a2 = (x_his(3,8) - x_his(3,7))/dt;
  
  ekf_.x_(2) = (x_lowpass(0,9) + ekf_.x_(2))/2;
  ekf_.x_(3) = (x_lowpass(1,9) + ekf_.x_(3))/2;
  
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
