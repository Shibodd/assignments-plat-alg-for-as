
#include "gauss.hpp"
#include <iostream>

int main() {
  Eigen::Vector2d x(0.351, -0.1);
  Eigen::Vector2d mean(5, 3);
  Eigen::Matrix2d cov;

  cov << 7, 2, 1, 3;

  double ans = gauss::multivariate_gauss_pdf(x, mean, cov);

  std::cerr << ans << std::endl;
}