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
	x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;
}


void KalmanFilter::__update(const Eigen::VectorXd &y) {
	MatrixXd S = H_ * P_ * H_.transpose() + R_;
	MatrixXd K = P_ * H_.transpose() * S.inverse();

	x_ = x_ + K * y;
	int x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::Update(const VectorXd &z) {
	__update(z - H_ * x_);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	float pi=atan(1)*4;

	float rho = z(0); // Range
	float phi = z(1); // Heading
	float rho_dot = fabs(rho) < 0.0001? 0 : z(2); // Radial velocity
	
	VectorXd z_pred(3);
	z_pred<<rho, phi, rho_dot;

	Eigen::Vector2d actual_position = x_.head(2);
	Eigen::Vector2d actual_velocity = x_.tail(2);
	double actual_range = actual_position.norm();

	VectorXd y(3);
	y << actual_range,
	  	 std::tan(actual_position(1) / actual_position(0)), // std::atan2(x_(1), x_(0))
			 actual_position.dot(actual_velocity);
		
	if(y[1]>=pi){ //Normalizing Angles
		y[1]=y[1]-(2*pi);
	}else if(y[1]<-pi){
		y[1]=y[1]+(2*pi);
	}

	__update(y);
}

