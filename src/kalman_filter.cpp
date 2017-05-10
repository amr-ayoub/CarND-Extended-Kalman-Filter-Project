#include "kalman_filter.h"
#include <math.h> 


using Eigen::MatrixXd;
using Eigen::VectorXd;

#define PI 3.1415926535897

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
	x_ = F_ * x_;
  	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	
  	VectorXd y = z - H_ * x_;
	
  	MatrixXd Ht = H_.transpose();
  	MatrixXd S = H_ * P_ * Ht + R_;
  	MatrixXd Si = S.inverse();
  	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;
	
	//estimate
	x_ = x_ + (K * y);
  	long x_size = x_.size();
  	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	
	float px = x_[0];
    float py = x_[1];
    float vx = x_[2];
    float vy = x_[3];

    float rho = sqrt(px * px + py * py);
    float phi = atan2(py,  px);

    if(rho < 1e-6) 
		rho = 1e-6;
	
    float rho_dot = (px * vx + py * vy) / rho;

    VectorXd z_pred(3);
    z_pred << rho, phi, rho_dot;

	VectorXd y = z - z_pred;
	
	while(y[1]> PI || y[1] < -PI)
    {
        if(y[1] > PI)
            y[1]-= PI;
        else y[1]+= PI;
	}
	
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
