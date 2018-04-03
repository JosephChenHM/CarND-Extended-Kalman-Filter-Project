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
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//compare result with requirement of submition
	if( rmse(0) > .11 || rmse(1) > .11 ||
		rmse(2) > .52 || rmse(3) > .52 )
    cout //<< "Timestep " << t << endl
         << "Exceeds requirement" << endl
         << ".11, .11, .52, .52" << endl
    	 << "RMSE = " 
         << rmse(0) << "  " << rmse(1) << "  " 
         << rmse(2) << "  " << rmse(3) << endl;
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
	double px = x_state(0);
	double py = x_state(1);
	double vx = x_state(2);
	double vy = x_state(3);
	//precompute
	double px_2 = px*px;
	double py_2 = py*py;
	double ppx = px/sqrt(px_2+py_2);
	double ppy = py/sqrt(px_2+py_2);
	//check division by zero
	if (px_2+py_2 < 0.0001)
	{
	    cout << "divide by zero" << endl;
	    return Hj;
	}
	Hj << ppx, ppy, 0, 0,
	     -1*py/(px_2+py_2), px/(px_2+py_2), 0, 0,
	     (py*(vx*py-vy*px))/pow(px_2+py_2,3./2), (px*(vy*px-vx*py))/pow(px_2+py_2,3./2), ppx, ppy;
    // cout << "Hj_ = " << Hj << endl;
	return Hj;
}
