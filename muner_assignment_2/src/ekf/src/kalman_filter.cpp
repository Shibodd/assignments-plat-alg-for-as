#include <iostream>
#include "kalman_filter.h"
#include "utils.hpp"


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
	Eigen::Vector2d actual_position = x_.head(2);
	Eigen::Vector2d actual_velocity = x_.tail(2);

	// The current state in the RADAR's measurement space
	double actual_range = actual_position.norm();
	double actual_heading = std::atan2(actual_position(1), actual_position(0));
	double actual_radial_velocity = actual_position.dot(actual_velocity) / actual_range; // Radial component of actual_velocity

	double measured_range = z(0);
	double measured_heading = z(1);
	double measured_radial_velocity = fabs(z(0)) < 0.0001? 0 : z(2); // If the range is too low, then override the radial velocity to 0

	// The difference between the actual state and measured state in the RADAR's space
	Eigen::Vector3d y(
		measured_range - actual_range,
		// Don't just perform a subtraction...
		// what's the difference between 175deg and -175deg? 10deg, not 350deg.
		// what's the difference between -175deg and 175deg? -10deg
		utils::angle_difference_radians(actual_heading, measured_heading),
		measured_radial_velocity - actual_radial_velocity
	);

	__update(y);
}