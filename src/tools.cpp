#include <iostream>
#include "tools.h"
#include <iostream>

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	VectorXd rmse(4);
	rmse << 0,0,0,0;


  /**
  TODO:
    * Calculate the RMSE here.
  */
	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	if (estimations.size() == 0) {
	   	cout << "RMSE - Error - estimation vector is Zero" << endl;
		return rmse;
	}
	//  * the estimation vector size should equal ground truth vector size
	// ... your code here
    	else if (estimations.size() != ground_truth.size()) {
        		cout << "RMSE - Error - estimation size is equal to ground truch size" << endl;
    	}
	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
        		// ... your code here
        		VectorXd res = estimations[i] - ground_truth[i];
        		//coefficient-wise multiplication
		res = res.array()*res.array();
		rmse += res;
		
	}


	//calculate the mean
	// ... your code here
    	int n = estimations.size();
    	rmse = rmse/n;
    
	//calculate the squared root
	// ... your code here
    	rmse = rmse.array().sqrt();
	//return the result
	return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

    	float pxy_sum = px * px + py * py;
    	float pxy_sumSqrt = sqrt(pxy_sum);
    
	//TODO: YOUR CODE HERE 
    	//x_state = (px, py, vx, vy)
	//check division by zero
	if(fabs(pxy_sum) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		Hj << 0, 0, 0, 0,
			0, 0, 0, 0,
			0, 0, 0, 0;
		return Hj;
	}
	//compute the Jacobian matrix  
	Hj << px / pxy_sumSqrt, py/pxy_sumSqrt, 0, 0,
	    -py/pxy_sum, px/pxy_sum, 0, 0,
	    py*(vx*py - vy*px)/(pxy_sum * pxy_sumSqrt), px*(vy*px - vx*py)/(pxy_sum * pxy_sumSqrt), px/pxy_sumSqrt, py/pxy_sumSqrt;
	
	return Hj;


}
