#ifndef STACKED_VECTORS_HPP
#define STACKED_VECTORS_HPP

#include <vector>
#include <eigen3/Eigen/Dense>

namespace stackedvec {

template <typename Scalar, int Dim>
static inline Eigen::Matrix<Scalar, Dim, -1> hstack(std::vector<Eigen::Matrix<Scalar, Dim, 1>> vectors) {
  Eigen::MatrixXd mat(Dim, vectors.size());
  for (size_t i = 0; i < vectors.size(); ++i)
    mat.col(i) = vectors[i];
  return mat;
}

template <typename Scalar, int Dim, int Mode>
static inline Eigen::Matrix<Scalar, Dim, -1> apply_transform(
    const Eigen::Transform<Scalar, Dim, Mode>& transform,
    const Eigen::Matrix<Scalar, Dim, -1>& vectors) {

  // Apply the transformation to each vector
  return (transform * vectors.colwise().homogeneous()).eval();
}

} // namespace stackedvec

#endif