#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

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
	x_ = ;
	MatrixXd Ft = F_.transpose();
	P_ = ;
}

void KalmanFilter::Update(const VectorXd &z) {
	/**
	TODO: 
	* update the state by using Kalman Filter equations
	*/
	VectorXd y = ;
	MatrixXd Ht = ;//use the transpose() function
	MatrixXd S = ;
	MatrixXd Si = ; //use the inverse() function
	MatrixXd PHt = ;
	MatrixXd K = ;

	/**
	TODO: 
	* Compute the new estimate
	*/
	x_ = ;
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = ;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/**
	TODO: 
	* update the state by using Extended Kalman Filter equations
	*/
  	float pi=atan(1)*4;

	float rho=;
	float phi=;
	
	float rho_dot;
	if (fabs(rho)<0.0001){
		rho_dot=0;	
	}else{
		rho_dot=;
	}
	VectorXd z_pred(3);
	z_pred<<rho, phi, rho_dot;

	VectorXd y = ;
	
	if(y[1]>=pi){ //Normalizing Angles
		y[1]=y[1]-(2*pi);
	}else if(y[1]<-pi){
		y[1]=y[1]+(2*pi);
	}
	
	MatrixXd Ht = ; //use the transpose() function
	MatrixXd S = ;
	MatrixXd Si = ; //use the inverse() function
	MatrixXd PHt = 
	MatrixXd K = ;

	//new estimate
	x_ = ;
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = ;

}

