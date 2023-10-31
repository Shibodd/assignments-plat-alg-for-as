#ifndef TRACKER_H_
#define TRACKER_H_

#include "tracker/Tracklet.h"
#include "viewer/Renderer.h"
#include <limits>
#include <functional>

using TracksUpdateListener = std::function<void(Tracklet&, bool)>;

class Tracker
{
public:
  Tracker();
  ~Tracker();

  // Dead tracklets removal logic
  void removeTracks(const std::vector<int> &det_association_vector);

  // New tracklets addition logic
  void addTracks(const std::vector<int> &det_association_vector,
                 const std::vector<double> &centroids_x,
                 const std::vector<double> &centroids_y);

  // Entry point
  void track(const std::vector<double> &centroids_x,
             const std::vector<double> &centroids_y,
             viewer::Renderer& renderer);

  inline void setTracksUpdateListener(TracksUpdateListener listener) { tracksUpdateListener_ = listener; }

  const std::vector<Tracklet> &getTracks() { return tracks_; }

private:
  TracksUpdateListener tracksUpdateListener_;
  // tracklets
  std::vector<Tracklet> tracks_;
  int cur_id_;

  // thresholds
  double distance_threshold_;
  int lost_count_threshold_;

  // Computes the assignment cost matrix for the data association problem.
  Eigen::MatrixXd assignment_cost_matrix(const std::vector<double> &det_xs, const std::vector<double> &det_ys) const;

  // Performs data association and returns the detection association vector
  std::vector<int> dataAssociation(const std::vector<double> &det_xs, const std::vector<double> &det_ys) const;
};

#endif // TRACKER_H_
