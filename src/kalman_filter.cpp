#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

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
/*
	 * KF Prediction step
	 */
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;

	//cout << "During prediction, x_:" << x_ << endl;
	//cout << "Ft:" << Ft << endl;
	//cout << "P_:" << P_ << endl;


}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
/*
	 * KF Measurement update step
	 */

     cout << "Before update, x_:  " << endl << x_ << endl;
	cout << "Before update, P_:  " << endl << P_ << endl;

	//measurement covariance matrix - laser
	R_laser_ = MatrixXd(2,2);
  	R_laser_ << 0.0225, 0,
        0, 0.0225;

	MatrixXd R_ = R_laser_;

	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_laser_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	cout << "H_ = " << endl << H_ << endl;
	cout << "z_pred = " << endl << z_pred << endl;
	cout << "y = " << endl << y << endl;
	cout << "P_ = " << endl << P_ << endl;
	cout << "Ht = " << endl << Ht << endl;


	cout << "R_laser_ = " << endl << R_laser_ << endl;
	
	cout << "S = " << endl << S << endl;
	cout << "PHt = " << endl << PHt << endl;
	cout << "K = " << endl << K << endl;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

	cout << "During update, H_:" << H_ << endl;
	cout << "y:" << y << endl;
	cout << "S:" << S << endl;

	cout << "PHt:" << PHt << endl;
	cout << "K:" << K << endl;
	cout << "P_:" << P_ << endl;


}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	// use Hj instead of H in EKF update
	Tools tools;
	//cout << "during EKF update, x_ = " << endl << x_ << endl;
	
	MatrixXd Hj = tools.CalculateJacobian(x_); 
	//cout << "Hj = " << endl << Hj << endl;
	Hx_= MatrixXd(3,1);
	//check division by zero when px*px + py*py is zero
	if(fabs(x_[0]*x_[0] + x_[1]*x_[1]) < 0.0001){
		cout << "Calculate H(x_) - Error - Division by Zero" << endl;
		Hx_ << 0, 0, 0; //0, 3.14/4, 0; 
	}
	else {
		Hx_ << sqrt(x_[0]*x_[0] + x_[1]*x_[1]), atan2(x_[1],x_[0]), ((x_[0]*x_[2] + x_[1]*x_[3])/sqrt(x_[0]*x_[0] + x_[1]*x_[1]));
	}

	//cout << "H(x_)= " << endl << Hx_<< endl;
	VectorXd y = z - Hx_;
	//cout << "z = " << endl << z << endl;
	//cout << "y = " << endl << y << endl;
	//cout << "P_ = " << endl << P_ << endl;
	MatrixXd Ht = Hj.transpose();
	//cout << "Ht = " << endl << Ht << endl;
	//measurement covariance matrix - radar
	R_radar_ = MatrixXd(3,3);
	R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

	MatrixXd R_ = R_radar_;
	//cout << "R_radar_ = " << endl << R_radar_ << endl;
	MatrixXd S = Hj * P_ * Ht + R_radar_;
	//cout << "S = " << endl << S << endl;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	//cout << "PHt = " << endl << PHt << endl;
	MatrixXd K = PHt * Si;
	//cout << "K = " << endl << K << endl;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * Hj) * P_;

	//cout << "During EKFUpdate, Hj:" << Hj << endl;	cout << "y:" << y << endl;
	//cout << "S:" << S << endl;

	//cout << "PHt:" << PHt << endl;
	//cout << "K:" << K << endl;
	//cout << "P_:" << P_ << endl;


}
