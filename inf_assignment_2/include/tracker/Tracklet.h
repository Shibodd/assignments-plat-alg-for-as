#ifndef TRACKLET_H_
#define TRACKLET_H_

#include "KalmanFilter.h"
#include <functional>

class Tracklet;
using HistoryListener = std::function<void(Tracklet&)>;


class Tracklet
{
public:
  Tracklet(int idTrack, double x, double y);
  ~Tracklet();

  void predict();
  void update(double x, double y, bool lidarStatus);

  // Records the path this tracklet has taken.
  void record();

  // getters
  inline double getX() const { return kf_.getX(); }
  inline double getY() const { return kf_.getY(); }
  inline Eigen::Vector2d getPosition() const { return kf_.getPosition(); }
  inline Eigen::Matrix2d getPositionCovariance() const { return kf_.getPositionCovariance(); }
  inline double getXCovariance() const { return kf_.getXCovariance(); }
  inline double getYCovariance() const { return kf_.getYCovariance(); }
  inline int getId() const { return id_; }
  inline int getLostCount() const { return lost_count_; }
  inline void increaseLostCount() { lost_count_++; }
  inline void resetLostCount() { lost_count_ = 0; }
  inline void setHistoryListener(HistoryListener f) { history_listener_ = f; }
  inline Eigen::Vector2d getLastPosition() { return history_last_position_; }

private:
  int history_length_;
  Eigen::Vector2d history_last_position_;
  HistoryListener history_listener_;

  // filter
  KalmanFilter kf_;

  // tracklet id
  int id_;

  int lost_count_;

  double distance_traveled_;
  double record_distance_threshold_;
};

#endif // TRACKLET_H_
