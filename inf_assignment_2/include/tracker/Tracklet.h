#ifndef TRACKLET_H_
#define TRACKLET_H_

#include <vector>
#include <cmath>

#include "KalmanFilter.h"

class Tracklet
{
public:
  Tracklet(int idTrack, double x, double y);
  ~Tracklet();

  void predict();
  void update(double x, double y, bool lidarStatus);

  // getters
  inline double getX() const { return kf_.getX(); }
  inline double getY() const { return kf_.getY(); }
  inline Eigen::Vector2d getPosition() const { return kf_.getPosition(); }
  inline Eigen::Matrix2d getPositionCovariance() const { return kf_.getPositionCovariance(); }
  double getXCovariance() { return kf_.getXCovariance(); }
  double getYCovariance() { return kf_.getYCovariance(); }
  int getId() { return id_; }

private:
  // filter
  KalmanFilter kf_;

  // tracklet id
  int id_;
};

#endif // TRACKLET_H_
