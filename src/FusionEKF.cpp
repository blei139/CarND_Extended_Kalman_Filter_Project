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
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement 
  */
	//create a 4D state vector, we don't know yet the values of the x state
	ekf_.x_ = VectorXd(4);

	//state covariance matrix P
	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1000, 0,
			  0, 0, 0, 1000;

	//measurement covariance
	//Laser 
	ekf_.R_ = MatrixXd(2, 2);
	ekf_.R_ << 0.0225, 0,
			  0, 0.0225; 
	
	//measurement matrix
	ekf_.H_ = MatrixXd(2, 4);
	ekf_.H_ << 1, 0, 0, 0,
			  0, 1, 0, 0;

	//the initial transition matrix F_
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 1, 0,
			  0, 1, 0, 1,
			  0, 0, 1, 0,
			  0, 0, 0, 1;

	//set the acceleration noise components
	noise_ax = 5;
	noise_ay = 5;

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
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    //cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
	//cout << "RADAR" << endl;
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
	//x_ is a 1x4 matrix of px, py, vx, and vy
	//Radar measurement is a 1x3 matrix of rho, phi, 
	//and rho dot 
	float rho = measurement_pack.raw_measurements_[0];
	float phi = measurement_pack.raw_measurements_[1];
	float rhoDot = measurement_pack.raw_measurements_[2];
	//set the state with the initial location and zero velocity
	float px = rho * cos(phi); //sqrt((rho*rho) / (tan(phi)*tan(phi) + 1));
	float py = rho * sin(phi); //tan(phi) * px;
	ekf_.x_ <<  px, py, 0, 0; //8.6, 0.25, -3.00029, 0; //rho * cos(phi), rho * sin(phi), sqrt(8.6*8.6+0.25*0.25)*cos(phi), sqrt(8.6*8.6+0.25*0.25)*sin(phi); //rho * cos(phi), rho * sin(phi), 0, 0;
	//cout << "rho: " << endl << rho << endl;
	//cout << "phi: " << endl << phi << endl;
	//cout << "rhoDot: " << endl << rhoDot << endl;

	//cout << "x_: " << endl << ekf_.x_ << endl;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
	//set the state with the initial location and zero velocity
	ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
	//cout << "Laser: " << endl;
	//cout << "x_" << endl << ekf_.x_ << endl;
    }


	previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
	//compute the time elapsed between the current and previous measurements
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;

    	// TODO: YOUR CODE HERE
	//1. Modify the F matrix so that the time is integrated
	ekf_.F_ << 1, 0, dt, 0,
			  0, 1, 0, dt,
			  0, 0, 1, 0,
			  0, 0, 0, 1;
	//std::cout << "F: " << ekf_.F_ << std::endl;

	noise_ax = 9; //for debugging purpose 5;
	noise_ay = 9; //for debugging purpose 5;

//2. Set the process covariance matrix Q
	// covariance square with respect to ax = noise_ax
	// covariance square with respect to ay = noise_ay
	float dt_square = dt * dt;
	float dt_cube = dt_square * dt;
	float dt_fourth = dt_cube * dt;
	//cout << "dt square: " << endl << dt_square << endl;
	//cout << "dt cube: " << endl << dt_cube << endl;
	//cout << "dt to the fourth: " << endl << dt_fourth << endl;

//Modify the F matrix so that the time is integrated
	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;

	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ << dt_fourth/4 * noise_ax, 0, dt_cube/2 * noise_ax, 0,
	            0, dt_fourth/4 * noise_ay, 0, dt_cube/2 * noise_ay,
	            dt_cube/2 * noise_ax, 0, dt_square * noise_ax, 0,
	            0, dt_cube/2 * noise_ay, 0, dt_square * noise_ay;
	//std::cout << "Q: " << ekf_.Q_ << std::endl;  
          
	//3. Call the Kalman Filter predict() function
  ekf_.Predict();


  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	//4. Call the Kalman Filter update() function
	// with the most recent raw measurements_
	
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
	//4. Call the Kalman Filter update() function
	// with the most recent raw measurements_
 
	ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
	//std::cout << "raw measurements: " << measurement_pack.raw_measurements_ << std::endl;
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;

}
