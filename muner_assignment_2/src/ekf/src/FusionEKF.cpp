#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include "ini.hpp"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF(const config::config_ty& cfg) {
	is_initialized_ = false;

	previous_timestamp_ = 0;

	// initializing matrices
	R_laser_ = MatrixXd(2, 2);
	R_radar_ = MatrixXd(3, 3);
	H_laser_ = MatrixXd(2, 4);
	Hj_ = MatrixXd(3, 4);

	//measurement covariance matrix - laser
	R_laser_ << cfg.laser_variance.x, 0,
		0, cfg.laser_variance.y;

	//measurement covariance matrix - radar
	R_radar_ << cfg.radar_variance.range, 0, 0,
				0, cfg.radar_variance.heading, 0,
				0, 0, cfg.radar_variance.range;
		
	//measurement matrix
	H_laser_ = MatrixXd(2, 4);
	H_laser_ << 1, 0, 0, 0,
			  0, 1, 0, 0;

	/**
	* Finish initializing the FusionEKF.
	* Set the process and measurement noises
	*/
	//create a 4D state vector, we don't know yet the values of the x state
	ekf_.x_ = VectorXd(4);

	// State covariance matrix P
	ekf_.P_ = MatrixXd::Identity(4, 4) * cfg.initial_variance.variance;

	//the initial transition matrix F_
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 1, 0,
			  0, 1, 0, 1,
			  0, 0, 1, 0,
			  0, 0, 0, 1;
	noise_ax = cfg.stochastic_noise.x;
	noise_ay = cfg.stochastic_noise.y;
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
		  * Initialize the state ekf_.x_ with the first measurement.
		  * Create the covariance matrix.
		  * Remember: you'll need to convert radar from polar to cartesian coordinates.
		*/
		// first measurement
		cout << "EKF: " << endl;
		ekf_.x_ = VectorXd(4);

		previous_timestamp_ = measurement_pack.timestamp_;
		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
			/**
			Convert radar from polar to cartesian coordinates and initialize state.
			*/
			float rho = measurement_pack.raw_measurements_[0];
			float phi = measurement_pack.raw_measurements_[1];
			float rho_dot = measurement_pack.raw_measurements_[2];
			ekf_.x_ << rho*cos(phi),rho*sin(phi),rho_dot*cos(phi),rho_dot*sin(phi);
		}
		else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
			/**
			Initialize state.
			*/
			ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
		}


		// done initializing, no need to predict or update
		is_initialized_ = true;
		return;
	}

	/*****************************************************************************
	*  Prediction
	****************************************************************************/
	/**
	 * Update the state transition matrix F according to the new elapsed time.
	  - Time is measured in seconds.
	*/
	//compute the time elapsed between the current and previous measurements
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;


	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;
	ekf_.F_ <<  1,  0, dt,  0,
			        0,  1,  0, dt,
			        0,  0,  1,  0,
			        0,  0,  0,  1;

	double c2 = dt_2;
	double c3 = dt_3 / 2;
	double c4 = dt_4 / 4;
	
	// Process covariance matrix Q
	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ <<  c4 * noise_ax, 0,             c3 * noise_ax, 0,
						  0,             c4 * noise_ay, 0,             c3 * noise_ay,
				      c3 * noise_ax, 0,             c2 * noise_ax, 0,
				      0,             c3 * noise_ay, 0,             c2 * noise_ay;

	ekf_.Predict();

	/*****************************************************************************
	*  Update
	****************************************************************************/

	/**
	 * Use the sensor type to perform the update step.
	 * Update the state and covariance matrices.
	*/


	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		Tools t_;
		ekf_.H_=t_.CalculateJacobian(ekf_.x_);
		ekf_.R_=R_radar_;
		ekf_.UpdateEKF(measurement_pack.raw_measurements_);
	} else {
		ekf_.H_=H_laser_;
		ekf_.R_=R_laser_;
		ekf_.Update(measurement_pack.raw_measurements_);
	}

	assert(not ekf_.x_.hasNaN());
	

	// print the  output
	// cout << "x_ = " << ekf_.x_ << endl;
	// cout << "P_ = " << ekf_.P_ << endl;

	// cout << endl;
}
