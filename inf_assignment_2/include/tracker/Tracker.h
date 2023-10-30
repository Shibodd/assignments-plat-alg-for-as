#ifndef TRACKER_H_
#define TRACKER_H_

#include "tracker/Tracklet.h"
#include <limits>

class Tracker
{
public:
  Tracker();
  ~Tracker();

  // handle tracklets
  void removeTracks();
  void addTracks(const std::vector<int> &det_associated_tracks,
                 const std::vector<double> &centroids_x,
                 const std::vector<double> &centroids_y);

  // track objects
  void track(const std::vector<double> &centroids_x,
             const std::vector<double> &centroids_y,
             bool lidarStatus);

  // getters
  const std::vector<Tracklet> &getTracks() { return tracks_; }

private:
  // tracklets
  std::vector<Tracklet> tracks_;
  int cur_id_;

  // thresholds
  double distance_threshold_2_;
  double covariance_threshold;

  // Computes the assignment cost matrix for the data association problem.
  Eigen::MatrixXd assignment_cost_matrix(const std::vector<double> &det_xs, const std::vector<double> &det_ys) const;

  // Performs data association and returns the detection association vector
  std::vector<int> dataAssociation(const std::vector<double> &det_xs, const std::vector<double> &det_ys) const;
};

#endif // TRACKER_H_
