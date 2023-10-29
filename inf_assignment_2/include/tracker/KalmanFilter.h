#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <eigen3/Eigen/Dense>

class KalmanFilter
{
public:
  KalmanFilter();
  ~KalmanFilter();

  // init the filter
  void init(double dt);

  void predict();
  void update(const Eigen::Vector2d& z);

  // setters
  void setState(double x, double y);

  // getters
  inline double getX() const { return x_(0); }
  inline double getY() const { return x_(1); }
  inline Eigen::Vector2d getPosition() const { return x_.head(2); }
  inline Eigen::Matrix2d getPositionCovariance() const { return P_.topLeftCorner(2, 2); }
  double getXCovariance() { return P_(0, 0); }
  double getYCovariance() { return P_(1, 1); }

private:
  // dt in seconds
  double dt_;

  // state vector
  Eigen::Vector4d x_;

  // state covariance matrix
  Eigen::Matrix4d P_;

  // state transition matrix
  Eigen::Matrix4d F_;

  // process covariance matrix
  Eigen::Matrix4d Q_;

  // measurement matrix
  Eigen::Matrix<double, 2, 4> H_;

  // measurement covariance matrix
  Eigen::Matrix2d R_;
};

#endif // KALMAN_FILTER_H_
