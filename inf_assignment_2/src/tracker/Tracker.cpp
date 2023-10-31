#include "tracker/Tracker.h"
#include "gauss.hpp"
#include "rectangular_lsap.hpp"
#include "logging.hpp"
#include <limits>

static const logging::Logger logger("tracker");


Tracker::Tracker()
{
  cur_id_ = 0;
  distance_threshold_ = 2; // meters
  lost_count_threshold_ = 50;
}


Tracker::~Tracker()
{
}


void Tracker::removeTracks(const std::vector<int> &det_association_vector)
{
  // Create a map to check if a track is 
  std::vector<bool> associated_tracks(tracks_.size());
  for (auto track_idx : det_association_vector) {
    if (track_idx >= 0) {
      associated_tracks[track_idx] = true;
    }
  }

  // For each track that is associated, reset the lost count.
  // For each track that is not associated, increase the lost count.
  for (int track_idx = 0; track_idx < tracks_.size(); ++track_idx) {
    auto& track = tracks_[track_idx];
    
    if (associated_tracks[track_idx]) {
      track.resetLostCount();
      logger.debug("Track %d is alive", track.getId());
    } else {
      track.increaseLostCount();
      logger.debug("Unassociated track %d - lost count %d", track.getId(), track.getLostCount());
    }
  }

  // Keep any track for which lost_count <= lost_count_threshold
  std::vector<Tracklet> tracks_to_keep;

  int lct = lost_count_threshold_;
  std::copy_if(tracks_.begin(), tracks_.end(), std::back_inserter(tracks_to_keep), [lct](Tracklet track) {
    bool keep = track.getLostCount() <= lct;
    if (!keep)
      logger.debug("Dropping track %d! - Lost count %d", track.getId(), track.getLostCount());
    return keep;
  });

  tracks_.swap(tracks_to_keep);
}


void Tracker::addTracks(const std::vector<int> &det_association_vector, const std::vector<double> &centroids_x, const std::vector<double> &centroids_y)
{
  assert(det_association_vector.size() == centroids_x.size() && centroids_x.size() == centroids_y.size());

  // For each non-associated detection, add a new tracklet
  for (size_t det_idx = 0; det_idx < det_association_vector.size(); ++det_idx) {
    if (det_association_vector[det_idx] < 0) {
      logger.debug("New track %d!", cur_id_);
      tracks_.push_back(Tracklet(cur_id_, centroids_x[det_idx], centroids_y[det_idx]));
      ++cur_id_;
    }
  }
}



Eigen::MatrixXd Tracker::assignment_cost_matrix(
    const std::vector<double> &det_xs,
    const std::vector<double> &det_ys) const {

  assert(det_xs.size() == det_ys.size());
  int det_count = det_xs.size();
  int track_count = tracks_.size();

  Eigen::MatrixXd ans(det_count, track_count);

  // Compute each element of the cost matrix
  for (size_t det_idx = 0; det_idx < det_count; ++det_idx) {
    Eigen::Vector2d det(det_xs[det_idx], det_ys[det_idx]);

    for (size_t tracklet_idx = 0; tracklet_idx < track_count; ++tracklet_idx) {
      const Tracklet& track = tracks_[tracklet_idx];

      auto pos = track.getPosition();
      auto cov = track.getPositionCovariance();
      double dist = gauss::multivariate_gauss_pdf(det, pos, cov);

      ans(det_idx, tracklet_idx) = -dist;
    }
  }

  return ans;
}

std::vector<int> Tracker::dataAssociation(const std::vector<double> &det_xs, const std::vector<double> &det_ys) const {
  assert(det_xs.size() == det_ys.size());

  // Compute the cost matrix for the Linear Sum Assignment Problem
  Eigen::MatrixXd association_costs = assignment_cost_matrix(det_xs, det_ys);

  // Solve the LSAP
  std::vector<std::pair<int, int>> associations = lsap::solve(association_costs);

  // Build the association vector
  // (for each detection, the track idx that it is associated to, or -1 if it is not associated)
  std::vector<int> association_vector(det_xs.size(), -1);
  for (auto association : associations) {
    int det_idx = association.first;
    int track_idx = association.second;

    auto track_pos = tracks_[track_idx].getPosition();
    auto det_pos = Eigen::Vector2d(det_xs[det_idx], det_ys[det_idx]);

    auto dist = (track_pos - det_pos).norm();
    
    if (dist > distance_threshold_)
      continue;

    association_vector[det_idx] = track_idx;
  }

  return association_vector;
}


void Tracker::track(const std::vector<double> &centroids_x,
                    const std::vector<double> &centroids_y,
                    viewer::Renderer& renderer)
{
  assert(centroids_x.size() == centroids_y.size());

  bool lidarStatus = renderer.getLidarStatus();

  // Predict each tracker
  for (auto& tracker : tracks_)
    tracker.predict();

  // Data association
  auto det_association_vector = dataAssociation(centroids_x, centroids_y);

  // Update the associated track for each associated detection
  for (int det_idx = 0; det_idx < det_association_vector.size(); ++det_idx)
  {
    auto track_idx = det_association_vector[det_idx];
    if (track_idx < 0)
      continue;

    tracks_[track_idx].update(centroids_x[det_idx], centroids_y[det_idx], lidarStatus);
    renderer.addText(centroids_x[det_idx], centroids_y[det_idx], std::to_string(tracks_[track_idx].getId()), "track_idx", 0, 1, 0);
  }

  // Remove dead tracklet
  removeTracks(det_association_vector);

  // Add new tracklets
  addTracks(det_association_vector, centroids_x, centroids_y);

  // Tracker recording
  for (auto& tracker : tracks_)
    tracker.record();
}
