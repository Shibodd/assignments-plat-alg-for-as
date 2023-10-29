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
  inline Eigen::Vector2d getPosition() const { return kf_.getPosition(); }
  inline Eigen::Matrix2d getPositionCovariance() const { return kf_.getPositionCovariance(); }
  double getXCovariance() { return kf_.getXCovariance(); }
  double getYCovariance() { return kf_.getYCovariance(); }
  int getLossCount() { return loss_count_; }
  int getId() { return id_; }

private:
  // filter
  KalmanFilter kf_;

  // tracklet id
  int id_;

  // number of loss since last update
  int loss_count_;
};

#endif // TRACKLET_H_
