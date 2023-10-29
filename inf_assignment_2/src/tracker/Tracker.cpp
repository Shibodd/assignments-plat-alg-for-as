#include "tracker/Tracker.h"
#include "gauss.hpp"
#include "rectangular_lsap.hpp"

Tracker::Tracker()
{
  cur_id_ = 0;
  distance_threshold_ = 0.0; // meters
  covariance_threshold = 0.0;
}

Tracker::~Tracker()
{
}

/*
    This function removes tracks based on any strategy
*/
void Tracker::removeTracks()
{
  std::vector<Tracklet> tracks_to_keep;

  for (size_t i = 0; i < tracks_.size(); ++i)
  {
    // TODO
    // Implement logic to discard old tracklets
    // logic_to_keep is a dummy placeholder to make the code compile and should be subsituted with the real condition
    bool logic_to_keep = true;
    if (logic_to_keep)
      tracks_to_keep.push_back(tracks_[i]);
  }

  tracks_.swap(tracks_to_keep);
}

/*
    This function add new tracks to the set of tracks ("tracks_" is the object that contains this)
*/
void Tracker::addTracks(const std::vector<bool> &associated_detections, const std::vector<double> &centroids_x, const std::vector<double> &centroids_y)
{
  // Adding not associated detections
  for (size_t i = 0; i < associated_detections.size(); ++i)
    if (!associated_detections[i])
      tracks_.push_back(Tracklet(cur_id_++, centroids_x[i], centroids_y[i]));
}



Eigen::MatrixXd Tracker::assignment_cost_matrix(
    const std::vector<double> &det_xs,
    const std::vector<double> &det_ys) const {

  assert(det_xs.size() == det_ys.size());
  int det_count = det_xs.size();
  int track_count = tracks_.size();

  Eigen::MatrixXd ans(track_count, det_count);
  for (size_t det_idx = 0; det_idx < det_count; ++det_idx) {
    Eigen::Vector2d det(det_xs[det_idx], det_ys[det_idx]);

    for (size_t tracklet_idx = 0; tracklet_idx < track_count; ++tracklet_idx) {
      const Tracklet& track = tracks_[tracklet_idx];

      // Use the square of the mahalanobis distance for performance
      ans(det_idx, tracklet_idx) = gauss::mahalanobis2(det, track.getPosition(), track.getPositionCovariance());
    }
  }

  return ans;
}


void Tracker::track(const std::vector<double> &centroids_x,
                    const std::vector<double> &centroids_y,
                    bool lidarStatus)
{
  assert(centroids_x.size() == centroids_y.size());

  // Predict each tracker
  for (auto tracker : tracks_)
    tracker.predict();

  // Data association
  Eigen::MatrixXd association_costs = assignment_cost_matrix(centroids_x, centroids_y);
  std::vector<std::pair<int, int>> associations = lsap::solve(association_costs);
  std::vector<bool> associated_detections(centroids_x.size(), false);

  for (auto association : associations)
  {
    auto det_id = association.first;
    auto track_id = association.second;

    // Update tracklets with the new detections
    tracks_[track_id].update(centroids_x[det_id], centroids_y[det_id], lidarStatus);

    // Mark this detection as associated (see Add new tracklets)
    associated_detections[det_id] = true;
  }

  // Remove dead tracklets
  removeTracks();

  // Add new tracklets
  addTracks(associated_detections, centroids_x, centroids_y);
}
