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
	
	VectorXd RMSE(4);
	RMSE << 0, 0, 0, 0;
	
	//  Checking the estimation vector size is not zero
	if (estimations.size() == 0)
	{
		std::cout << "Invalid estimation" << std::endl;
		return RMSE;
	}
	
	
	//  Checking the estimation vector size equals ground truth vector size
	if (estimations.size() != ground_truth.size())
	{
		std::cout << "The estimation vector size doesn't equal ground truth vector size" << std::endl;
		return RMSE;
	}
	
	// Residuals
	for (unsigned int i = 0; i < estimations.size(); ++i)
	{

      VectorXd residual = estimations[i] - ground_truth[i];

      residual = residual.array()*residual.array();
      RMSE += residual;
	}
		
	// Mean
	RMSE = RMSE / estimations.size();
	
	// Squared root
	RMSE = RMSE.array().sqrt();
	
	// Result
	return RMSE;
}
	

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	
	MatrixXd Hj(3, 4);
	
  	//state values
  	float px = x_state(0);
  	float py = x_state(1);
  	float vx = x_state(2);
	float vy = x_state(3);
	
	//check division by zero
  	if (fabs(px*px + py*py) < 0.0001)
	{
      std::cout << "Division by zero error" << std::endl;
      return Hj;
	}
	
	float c1 = px*px + py*py;
  	float c2 = sqrt(px*px + py*py);
	float c3 = (c1*c2);
	
	//Jacobian matrix
  	Hj << (px / c2), (py / c2), 0, 0,
      -(py / c1), (px / c1), 0, 0,
      py*(vx*py - vy*px) / c3, px*(px*vy - py*vx) / c3, px / c2, py / c2;
	
	return Hj;

	
}
