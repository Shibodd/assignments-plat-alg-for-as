#include "tracker/Tracker.h"
#include "gauss.hpp"
#include "rectangular_lsap.hpp"
#include "logging.hpp"
#include <limits>

static const logging::Logger logger("tracker");

static const double HUGE = std::numeric_limits<double>::max();

Tracker::Tracker()
{
  cur_id_ = 0;
  distance_threshold_2_ = 0.0; // meters
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
void Tracker::addTracks(const std::vector<int> &det_association_vector, const std::vector<double> &centroids_x, const std::vector<double> &centroids_y)
{
  assert(det_association_vector.size() == centroids_x.size() && centroids_x.size() == centroids_y.size());

  // Adding not associated detections
  for (size_t det_idx = 0; det_idx < det_association_vector.size(); ++det_idx)
    if (det_association_vector[det_idx] < 0)
      tracks_.push_back(Tracklet(cur_id_++, centroids_x[det_idx], centroids_y[det_idx]));
}



Eigen::MatrixXd Tracker::assignment_cost_matrix(
    const std::vector<double> &det_xs,
    const std::vector<double> &det_ys) const {

  assert(det_xs.size() == det_ys.size());
  int det_count = det_xs.size();
  int track_count = tracks_.size();

  Eigen::MatrixXd ans(det_count, track_count);
  for (size_t det_idx = 0; det_idx < det_count; ++det_idx) {
    Eigen::Vector2d det(det_xs[det_idx], det_ys[det_idx]);

    for (size_t tracklet_idx = 0; tracklet_idx < track_count; ++tracklet_idx) {
      const Tracklet& track = tracks_[tracklet_idx];

      // Use the square of the mahalanobis distance for performance
      auto pos = track.getPosition();
      auto cov = track.getPositionCovariance();
      double mah = gauss::mahalanobis2(det, pos, cov);

      // If the distance is too large, set a huge distance to penalize the association
      if ((pos - det).squaredNorm() > distance_threshold_2_)
        mah = HUGE;
      
      ans(det_idx, tracklet_idx) = mah;
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

    // See assignment_cost_matrix
    // Penalizing an association doesn't mean it will never be picked
    if (association_costs(det_idx, track_idx) >= HUGE)
      continue; 

    association_vector[det_idx] = track_idx;
  }

  return association_vector;
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
  auto det_association_vector = dataAssociation(centroids_x, centroids_y);  

  for (int det_idx = 0; det_idx < det_association_vector.size(); ++det_idx)
  {
    // For each associated detection
    auto track_idx = det_association_vector[det_idx];
    if (track_idx < 0)
      continue;

    // Update the associated track using the new detection
    tracks_[track_idx].update(centroids_x[det_idx], centroids_y[det_idx], lidarStatus);
  }

  // Remove dead tracklets
  removeTracks();

  // Add new tracklets
  addTracks(det_association_vector, centroids_x, centroids_y);
}
