#include "tracker/Tracklet.h"
#include "logging.hpp"

static logging::Logger logger("tracklet");


Tracklet::Tracklet(int idTrack, double x, double y)
  : lost_count_(0),
    id_(idTrack),
    history_length_(0),
    history_listener_(0),
    record_distance_threshold_(0.1),
    distance_traveled_(0)
{
  // initialize filter
  kf_.init(0.1);
  kf_.setState(x, y);
}

Tracklet::~Tracklet()
{
}

void Tracklet::record()
{
  auto rec = [this](Eigen::Vector2d pt) {
    this->history_last_position_ = pt;
    if (this->history_listener_)
      this->history_listener_(*this);
    ++this->history_length_;
  };

  Eigen::Vector2d position = kf_.getPosition();

  // If this is the first position we try to record, then record it
  if (history_length_ <= 0) {
    logger.info("Recording first point for %d.", this->getId());
    rec(position);
    distance_traveled_ = 0;
    return;
  }

  // If the distance from the current position to the
  // last recorded position is greater than threshold, record
  double dist = (history_last_position_ - position).norm();
  if (dist > record_distance_threshold_) {
    logger.info("Recording point no. %d for %d (total distance %.1fm).", history_length_, this->getId(), distance_traveled_);
    rec(position);
    distance_traveled_ += dist;
  }
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
