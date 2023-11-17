#include "data_association.hpp"
#include "gauss.hpp"
#include "rectangular_lsap.hpp"

static Eigen::MatrixXd assignment_cost_matrix(
    const std::vector<Eigen::Vector2d> &observations,
    const std::vector<Eigen::Vector2d> &map,
    const Eigen::Matrix2d covariance)
{

  int obs_count = observations.size();
  int map_count = map.size();

  Eigen::MatrixXd ans(obs_count, map_count);

  // Compute each element of the cost matrix
  for (size_t obs_idx = 0; obs_idx < obs_count; ++obs_idx)
  {
    for (size_t map_idx = 0; map_idx < map_count; ++map_idx)
    {
      double dist = gauss::mahalanobis2(observations[obs_idx], map[map_idx], covariance);
      ans(obs_idx, map_idx) = dist;
    }
  }

  return ans;
}

std::vector<std::pair<int, int>> point_association(
    const std::vector<Eigen::Vector2d> &observations,
    const std::vector<Eigen::Vector2d> &map,
    Eigen::Matrix2d covariance)
{
  // Compute the cost matrix for the Linear Sum Assignment Problem
  Eigen::MatrixXd association_costs = assignment_cost_matrix(observations, map, covariance);

  // Solve the LSAP
  return lsap::solve(association_costs);
}