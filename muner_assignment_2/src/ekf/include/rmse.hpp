#ifndef RMSE_HPP
#define RMSE_HPP

#include <vector>
#include <eigen3/Eigen/Dense>
#include <numeric>

namespace rmse {

  std::vector<double> calculatePositionSquaredErrors(const std::vector<Eigen::Vector2d>& estimates, const std::vector<Eigen::Vector2d>& ground_truth) {
    size_t n = estimates.size();
    assert(n == ground_truth.size());
    assert(n > 0);

    std::vector<double> ans;
    ans.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      double err = (ground_truth[i] - estimates[i]).squaredNorm();
      ans.push_back(err);
    }
    return ans;
  }

  double calculateRmse(const std::vector<double>& squared_errors) {
    double sum = std::accumulate(squared_errors.cbegin(), squared_errors.cend(), 0);
    size_t n = squared_errors.size();
    return std::sqrt(sum / n);
  }
} // namespace rmse

#endif // RMSE_HPP