#include "rectangular_lsap.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>

Eigen::MatrixXd assignment_cost_matrix(
    const std::vector<Eigen::Vector2d> &tracks,
    const std::vector<Eigen::Vector2d> &detections) {

  int det_count = detections.size();
  int track_count = tracks.size();

  Eigen::MatrixXd ans(det_count, track_count);

  // Compute each element of the cost matrix
  for (size_t det_idx = 0; det_idx < det_count; ++det_idx) {
    Eigen::Vector2d det = detections[det_idx];

    for (size_t tracklet_idx = 0; tracklet_idx < track_count; ++tracklet_idx) {
      Eigen::Vector2d track = tracks[tracklet_idx];
      double dist = 1 / (track - det).norm();

      ans(det_idx, tracklet_idx) = -dist;
    }
  }

  return ans;
}

int main() {
  std::vector<Eigen::Vector2d> tracks;
  #include "trackers.h"

  std::vector<Eigen::Vector2d> detections;
  #include "detections.h"

  auto costs = assignment_cost_matrix(tracks, detections);
  
  auto assignments = lsap::solve(costs);
  for (auto ass : assignments) {
    std::cout << ass.first << ", " << ass.second << std::endl;
  }
}