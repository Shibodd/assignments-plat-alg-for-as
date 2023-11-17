#ifndef GAUSS_HPP
#define GAUSS_HPP

#include <eigen3/Eigen/Dense>

namespace gauss {

/*
Returns the square of the mahalanobis distance from point x
to the normal distribution with the specified mean and covariance.
*/
template <typename _Scalar, int K>
inline double mahalanobis2(
    Eigen::Matrix<_Scalar, K, 1> x,
    Eigen::Matrix<_Scalar, K, 1> mean,
    Eigen::Matrix<_Scalar, K, K> covariance)
{
  Eigen::Matrix<_Scalar, K, 1> delta = x - mean;
  return delta.dot(covariance.inverse() * delta);
}

#define PI 3.14159265359

template <typename _Scalar, int K>
inline double multivariate_gauss_pdf(
    Eigen::Matrix<_Scalar, K, 1> x,
    Eigen::Matrix<_Scalar, K, 1> mean,
    Eigen::Matrix<_Scalar, K, K> covariance)
{
  double mah = mahalanobis2(x, mean, covariance);
  double c = std::pow(2 * PI, K);

  return std::exp(-mah / 2) / std::sqrt(c * covariance.determinant());
}

} // namespace gauss
#endif // !GAUSS_HPP