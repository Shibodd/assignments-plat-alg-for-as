#include "tracker/Tracklet.h"

Tracklet::Tracklet(int idTrack, double x, double y)
  : lost_count_(0),
    id_(idTrack)
{
  // initialize filter
  kf_.init(0.1);
  kf_.setState(x, y);
}

Tracklet::~Tracklet()
{
}

// Predict a single measurement
void Tracklet::predict()
{
  kf_.predict();
}

// Update with a real measurement
void Tracklet::update(double x, double y, bool lidarStatus)
{
  Eigen::Vector2d raw_measurements_ = Eigen::Vector2d(2);

  // measurement update
  if (lidarStatus)
  {
    raw_measurements_ << x, y;
    kf_.update(raw_measurements_);
  }
}
