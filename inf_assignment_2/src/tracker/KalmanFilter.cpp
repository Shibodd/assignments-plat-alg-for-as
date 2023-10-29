#include "tracker/KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::init(double dt)
{
  dt_ = dt;

  x_ << 0, 0, 0, 0;

  // state covariance matrix P
  P_ << 9999., 0., 0., 0.,
      0., 9999., 0., 0.,
      0., 0., 9999., 0.,
      0., 0., 0., 9999.;

  // measurement covariance
  R_ << 0.0225, 0.,
      0., 0.0225;

  // measurement matrix
  H_ << 1., 0., 0., 0.,
      0., 1., 0., 0.;

  // the transition matrix F
  F_ << 1., 0., dt_, 0.,
      0., 1., 0., dt_,
      0., 0., 1., 0.,
      0., 0., 0., 1.;

  // the acceleration noise components
  double noise_ax_ = 2.;
  double noise_ay_ = 2.;

  double dt_2 = dt_ * dt_;
  double dt_3 = dt_2 * dt_;
  double dt_4 = dt_3 * dt_;

  // the process covariance matrix Q
  Q_ << dt_4 / 4. * noise_ax_, 0., dt_3 / 2. * noise_ax_, 0.,
      0., dt_4 / 4. * noise_ay_, 0., dt_3 / 2. * noise_ay_,
      dt_3 / 2. * noise_ax_, 0., dt_2 * noise_ax_, 0.,
      0., dt_3 / 2. * noise_ay_, 0., dt_2 * noise_ay_;
}

void KalmanFilter::predict()
{
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose();
}

void KalmanFilter::update(const Eigen::VectorXd &z)
{
  Eigen::VectorXd y = z - H_ * x_;
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

  // new estimate
  x_ = x_ + (K * y);
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::setState(double x, double y)
{
  x_ << x, y, 0., 0.;
}